"""Process-parallel sampling. Workers simulate on CPU; learner stays on CUDA."""
import multiprocessing as mp
import traceback

import numpy as np
import torch

from realdata_train import RealDataHardwareEnv31, reward, SamplePolicy31, verified_artifact


def make_env(settings, balance=None):
    if settings['task']=='balance':
        return RealDataHardwareEnv31(episode_seconds=settings['episode_seconds'],config=settings.get('config'))
    from fifty_mujoco.envs.sample_env31_supported import SupportedGetupEnv31
    return SupportedGetupEnv31(balance,reset_range=settings['reset_range'],
                              settle_seconds=settings['settle_seconds'],domain=settings['domain'],config=settings.get('config'))


def worker(connection, ids, total, settings):
    try:
        torch.set_num_threads(1)
        balance=None
        if settings['task']!='balance':
            policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu')
            policy.load_state_dict(torch.load(settings['balance_checkpoint'],map_location='cpu',weights_only=True)['model'])
            balance=policy.actor.requires_grad_(False).eval()
        envs=[make_env(settings,balance) for _ in ids]
        episodes=[0]*len(ids)
        def reset(i):
            seed=settings['seed']+ids[i]+total*episodes[i]
            if not 510000<=seed<600000:raise RuntimeError('training seeds exhausted')
            episodes[i]+=1
            return envs[i].reset(seed)
        connection.send(('ready',None))
        while True:
            command,payload=connection.recv()
            if command=='close':break
            if command=='reset':
                connection.send(('ok',[reset(i) for i in range(len(ids))]));continue
            if command!='step':raise ValueError(command)
            result=[]
            for i,(env,action) in enumerate(zip(envs,payload)):
                previous=np.asarray(env.env.last_action if settings['task']!='balance' else env.last_action).copy()
                observation,value,terminated,truncated,info=env.step(action)
                if settings['task']=='balance':value=reward(env,info,action,previous)
                following=reset(i) if terminated or truncated else observation
                # Keep terminal observations intact for correct timeout bootstrap.
                result.append((observation,value,terminated,truncated,info,following))
            connection.send(('ok',result))
    except BaseException:
        try:connection.send(('error',traceback.format_exc()))
        except Exception:pass
    finally:
        connection.close()


class ProcessSampler:
    def __init__(self,num_envs,workers,settings):
        self.groups=[v.tolist() for v in np.array_split(np.arange(num_envs),min(workers,num_envs))]
        self.processes=[];self.connections=[]
        context=mp.get_context('spawn')
        try:
            for ids in self.groups:
                parent,child=context.Pipe()
                process=context.Process(target=worker,args=(child,ids,num_envs,settings),daemon=True)
                process.start();child.close();self.connections.append(parent);self.processes.append(process)
            for conn in self.connections:self._receive(conn,180)
        except BaseException:
            self.close();raise

    def _receive(self,connection,timeout=180):
        if not connection.poll(timeout):raise TimeoutError('sampling worker did not respond')
        status,payload=connection.recv()
        if status=='error':raise RuntimeError(payload)
        return payload

    def reset(self):
        for c in self.connections:c.send(('reset',None))
        return [row for c in self.connections for row in self._receive(c)]

    def step(self,actions):
        for c,ids in zip(self.connections,self.groups):c.send(('step',actions[ids]))
        return [row for c in self.connections for row in self._receive(c)]

    def close(self):
        for c in self.connections:
            try:c.send(('close',None))
            except Exception:pass
        for process in self.processes:
            process.join(timeout=3)
            if process.is_alive():process.terminate();process.join(timeout=3)
        for c in self.connections:c.close()
