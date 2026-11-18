"""Plot training cohorts and CPU evaluations for this run."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent


def rows(path):
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
ax = axes[0, 0]
for name in ('ppo_nominal_partial', 'ppo_mild_domain', 'ppo_smooth_mild'):
    data = rows(root/name/'rows.jsonl')
    if data:
        ax.plot([x['update'] for x in data], [100*x['cohort_successes']/x['cohort_trials'] for x in data],
                marker='.', label=name.replace('ppo_', '').replace('_', ' '))
ax.set(title='PPO training cohorts (GPU)', xlabel='Update', ylabel='Complete rises (%)')
ax.legend()

ax = axes[0, 1]
for label, name, fields in [
    ('Standard MLP, GPU', 'affine_mild', ('update','all_successes','worlds')),
    ('State-triggered sequence, GPU', 'contact_cem_mild', ('iteration','all_successes','resets')),
    ('Standard MLP, CPU (best per batch)', 'affine_cpu_mild', ('iteration','best','replicates')),
]:
    data = rows(root/name/'rows.jsonl')
    if data:
        step, wins, count = fields
        ax.plot([x[step] for x in data], [100*x[wins]/x[count] for x in data], marker='.', label=label)
ax.set(title='Search over policy parameters', xlabel='Iteration', ylabel='Complete rises (%)')
ax.legend()

ax = axes[1, 0]
data = []
for f in sorted((root/'dev_candidate_matrix').glob('*.json')):
    x=json.loads(f.read_text())
    if isinstance(x, dict) and 'passed' in x:
        data.append((f.stem.replace('_',' '), x['passed']/x['trials']*100))
if data:
    ax.barh([k for k,_ in data], [v for _,v in data])
    ax.set(xlim=(0,100), xlabel='Complete rises (%)', title='CPU development matrix')
    ax.tick_params(axis='y', labelsize=7)
else:
    ax.axis('off')

ax = axes[1, 1]
result = root/'final_summary.json'
if result.is_file():
    x=json.loads(result.read_text())
    names=[item['label'] for item in x['groups']]
    vals=[100*item['passed']/item['trials'] for item in x['groups']]
    ax.barh(names,vals)
    ax.axvline(95,linestyle='--',color='black',label='95% target')
    ax.set(xlim=(0,100),xlabel='Complete rises (%)',title='Frozen CPU holdout')
    ax.legend()
else:
    ax.axis('off')
fig.suptitle('Seated-back get-up | no external support')
fig.savefig(root/'training_progress.png',dpi=150)
