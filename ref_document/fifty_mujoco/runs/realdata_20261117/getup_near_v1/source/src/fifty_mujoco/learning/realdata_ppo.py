"""PPO with optional teacher constraint and ragged get-up transition masks."""
import copy

import torch
from torch import nn

from .sample_ppo import SamplePPO,gae


class RealDataPPO(SamplePPO):
    def __init__(self,policy,*args,anchor_coefficient=0.,**kwargs):
        super().__init__(policy,*args,**kwargs)
        self.anchor=copy.deepcopy(policy.actor).requires_grad_(False).eval()
        self.anchor_std=policy.log_std.exp().detach().clone()
        self.anchor_coefficient=anchor_coefficient

    def update(self,rollout,warmup=False,epochs=4,batch_size=4096):
        advantage,returns=gae(rollout['reward'],rollout['value'],rollout['next_value'],rollout['terminated'],rollout['truncated'])
        mask=rollout.get('valid',torch.ones_like(rollout['terminated'])).flatten()
        obs=rollout['observation'].reshape(-1,31)[mask]
        action=rollout['action'].reshape(-1,6)[mask]
        old_mean=rollout['mean'].reshape(-1,6)[mask];old_std=rollout['std'].reshape(-1,6)[mask]
        old_lp=rollout['log_prob'].flatten()[mask];target=returns.flatten()[mask]
        adv=advantage.flatten()[mask];adv=(adv-adv.mean())/(adv.std(unbiased=False)+1e-8)
        if len(obs)==0:raise ValueError('no valid learner transitions')
        with torch.no_grad():anchor=self.anchor(obs)
        totals={'value_loss':0.,'policy_loss':0.,'entropy':0.,'kl':0.,'anchor_loss':0.}
        batches=rollbacks=skips=0
        for _ in range(epochs):
            for index in torch.randperm(len(obs),device=obs.device).split(batch_size):
                lv=(self.policy.value(obs[index])-target[index]).square().mean()
                self.critic_optimizer.zero_grad();lv.backward()
                nn.utils.clip_grad_norm_(self.policy.critic.parameters(),1.,error_if_nonfinite=True)
                self.critic_optimizer.step()
                distribution=self.policy.distribution(obs[index]);lp=distribution.log_prob(action[index]).sum(-1)
                ratio=(lp-old_lp[index]).exp();loss=-torch.minimum(ratio*adv[index],ratio.clamp(.8,1.2)*adv[index]).mean()
                entropy=distribution.entropy().sum(-1).mean()
                constraint=((distribution.mean-anchor[index])/self.anchor_std).square().mean()
                with torch.no_grad():
                    kl=torch.distributions.kl_divergence(torch.distributions.Normal(old_mean[index],old_std[index]),distribution).sum(-1).mean()
                if not warmup and kl<=.02:
                    parameters=[*self.policy.actor.parameters(),self.policy.log_std]
                    before=[p.detach().clone() for p in parameters];optimizer_before=copy.deepcopy(self.actor_optimizer.state_dict())
                    self.actor_optimizer.zero_grad();(loss-.0001*entropy+self.anchor_coefficient*constraint).backward()
                    nn.utils.clip_grad_norm_(parameters,1.,error_if_nonfinite=True);self.actor_optimizer.step()
                    with torch.no_grad():
                        self.policy.log_std.clamp_(min=-6.9,max=0.)
                        full=torch.distributions.kl_divergence(torch.distributions.Normal(old_mean,old_std),self.policy.distribution(obs)).sum(-1).mean()
                        if not torch.isfinite(full) or full>.02:
                            for p,old in zip(parameters,before):p.copy_(old)
                            self.actor_optimizer.load_state_dict(optimizer_before)
                            for group in self.actor_optimizer.param_groups:group['lr']*=.5
                            rollbacks+=1
                elif not warmup:skips+=1
                for name,value in zip(totals,(lv,loss,entropy,kl,constraint)):totals[name]+=float(value.detach())
                batches+=1
        if not all(torch.isfinite(p).all() for p in self.policy.state_dict().values()):raise FloatingPointError('nonfinite policy')
        with torch.no_grad():
            final_kl=torch.distributions.kl_divergence(torch.distributions.Normal(old_mean,old_std),self.policy.distribution(obs)).sum(-1).mean()
        return {**{k:v/batches for k,v in totals.items()},'valid_transitions':len(obs),'critic_warmup':warmup,
                'actor_kl_rollbacks':rollbacks,'actor_kl_guard_skips':skips,'final_full_rollout_kl':float(final_kl),
                'actor_learning_rate':self.actor_optimizer.param_groups[0]['lr']}
