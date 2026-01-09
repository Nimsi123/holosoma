#!/bin/bash
#SBATCH --job-name=4dh-em1
#SBATCH --partition=Main
#SBATCH --nodelist=em1
#SBATCH --gres=gpu:1
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=50
#SBATCH --qos=low

eval "$(conda shell.bash hook)"

cd /home/nima/holosoma/src/holosoma_retargeting
bash parallel_ttg_retargeting_smplh.sh