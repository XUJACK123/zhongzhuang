"""Batched training counterpart of deployment's bounded command ramp.

Requested targets are distinct from applied commands. No filtering of actions,
sensor readings or physical state. Torch is injected for Isaac Gym import order.
"""
import math


class TrainingCommands:
    def __init__(self, template, rates, defaults, torch):
        if len(rates)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) or x<=0 for x in rates):
            raise ValueError('Three positive finite command rates required')
        if len(defaults)!=3 or any(not math.isfinite(x) for x in defaults):
            raise ValueError('Three finite default commands required')
        self.torch=torch
        self.rates=template.new_tensor(rates)
        self.defaults=template.new_tensor(defaults)
        self.applied=template.clone()
        self.applied[:]=self.defaults
        self.requested=self.applied.clone()

    def request(self, ids, values, reset=False):
        self.requested[ids]=values
        if reset:self.applied[ids]=self.defaults

    def advance(self, dt):
        if not math.isfinite(dt) or dt<=0:raise ValueError('Positive finite control interval required')
        step=self.rates*dt
        self.applied.add_(self.torch.maximum(-step,self.torch.minimum(step,self.requested-self.applied)))
        return self.applied


def audit(env,runner,path):
    """Exercise the real native callbacks and partial reset;not a skill test."""
    import json
    ramp=env.training_commands
    if ramp is None:raise ValueError('Training command ramp must be enabled')
    torch=ramp.torch;ids=torch.arange(env.num_envs,device=env.device)
    actor=runner.alg.actor_critic;actor.eval();rows=[]
    defaults=ramp.defaults.expand_as(ramp.applied)
    env.reset_idx(ids)
    torch.testing.assert_allclose(env.commands[:,:3],defaults,rtol=0,atol=1e-7)
    targets=env.commands[:,:3].clone()
    targets[::2]=targets.new_tensor([1.,.3,.14])
    targets[1::2]=targets.new_tensor([-1.,-.3,.08])
    with torch.no_grad():
        for tick in range(20):
            if tick==10:
                targets[:,:2].neg_();targets[:,2]=.22-targets[:,2]
            ramp.request(ids,targets)
            previous=env.commands[:,:3].clone()
            env.compute_observations()
            env.step(actor.act_inference(env.obs_buf))
            if env.reset_buf.any().item():raise RuntimeError('Unexpected physical reset during command audit')
            step=ramp.rates*env.dt
            expected=previous+torch.maximum(-step,torch.minimum(step,targets-previous))
            error=(env.commands[:,:3]-expected).abs().max().item()
            if error>2e-7:raise RuntimeError('Applied commands do not follow the specified ramp')
            rows.append(dict(tick=tick,max_error=error,requested=targets.cpu().tolist(),applied=env.commands[:,:3].cpu().tolist()))
    neighbor=env.commands[1:,:3].clone()
    env.reset_idx(ids[:1])
    torch.testing.assert_allclose(env.commands[0,:3],ramp.defaults,rtol=0,atol=1e-7)
    if not torch.equal(neighbor,env.commands[1:,:3]):raise RuntimeError('Partial reset changed neighboring commands')
    report=dict(passed=True,trials=env.num_envs,steps=20,dt=env.dt,rates=ramp.rates.cpu().tolist(),
                partial_reset_verified=True,rows=rows,note='Native command lifecycle plumbing only;not tracking acceptance or PPO training.')
    with path.open('x',encoding='utf-8') as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
