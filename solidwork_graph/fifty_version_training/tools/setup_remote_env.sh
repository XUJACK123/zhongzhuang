#!/usr/bin/env bash
# Prepare a remote Linux GPU host for the Fudan / fifty training stack.
#
#   bash tools/setup_remote_env.sh
#
# What it does
#   1. checks the GPU / driver and prints the compute capability
#   2. creates (or reuses) the conda env `leg38` with Python 3.8 + numpy 1.23.5
#   3. installs torch 2.4.1+cu118 and tensorboard
#   4. checks for Isaac Gym and prints the manual download instructions if absent
#
# It never downloads Isaac Gym (NVIDIA login required) and never touches the
# training repository.
set -euo pipefail

echo "== GPU =="
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,compute_cap,memory.total,driver_version --format=csv,noheader
else
  echo "nvidia-smi not found: this host has no usable NVIDIA driver." >&2
  exit 1
fi

echo
echo "== conda =="
if ! command -v conda >/dev/null 2>&1; then
  echo "conda not found. Install Miniconda first, then re-run this script." >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"

if conda env list | awk '{print $1}' | grep -qx leg38; then
  echo "env leg38 already exists"
else
  echo "creating env leg38 (conda-forge)"
  conda create -y -n leg38 -c conda-forge --override-channels python=3.8 numpy=1.23.5 pip
fi

echo
echo "== python packages =="
conda run -n leg38 python -m pip install --no-input \
  "torch==2.4.1" --index-url https://download.pytorch.org/whl/cu118
conda run -n leg38 python -m pip install --no-input "tensorboard>=2.11"

echo
echo "== verify =="
conda run -n leg38 python - <<'PY'
import numpy, torch
print("torch", torch.__version__, "| numpy", numpy.__version__)
print("cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    cap = torch.cuda.get_device_capability(0)
    print("device:", torch.cuda.get_device_name(0), "| compute capability:", cap)
    x = torch.randn(256, 256, device="cuda")
    print("matmul ok:", float((x @ x).sum()))
else:
    raise SystemExit("CUDA is not available -- install a matching driver before training.")
PY

echo
echo "== isaac gym =="
if conda run -n leg38 python -c "import isaacgym" >/dev/null 2>&1; then
  echo "isaacgym import OK"
else
  cat <<'EOF'
isaacgym is missing. It cannot be downloaded by script:
  1. log in at https://developer.nvidia.com/isaac-gym
  2. download IsaacGym_Preview_4_Package.tar.gz
  3. tar -xf IsaacGym_Preview_4_Package.tar.gz -C "$HOME"
  4. conda activate leg38
     cd "$HOME/isaacgym/python" && python -m pip install -e .
     export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
  5. cd "$HOME/isaacgym/python/examples" && python 1080_balls_of_solitude.py
EOF
  exit 2
fi

cat <<'EOF'

Next:
  cd <workspace>/solidwork_graph/fifty_version_training
  python integration/validate_model.py
  python integration/train_fifty.py --smoke-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main --headless
  python integration/train_fifty.py --joint-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main --headless
EOF
