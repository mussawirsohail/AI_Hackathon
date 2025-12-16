# install_torch_cpu.py
import subprocess
import sys

# Uninstall existing PyTorch packages
subprocess.run([sys.executable, "-m", "pip", "uninstall", "torch", "torchvision", "torchaudio", "-y"])

# Install CPU-only PyTorch
subprocess.run([sys.executable, "-m", "pip", "install",
                "torch", "torchvision", "torchaudio",
                "--index-url", "https://download.pytorch.org/whl/cpu"])
