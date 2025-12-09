source /bluesclues-data/home/pingpong-nima/.holosoma_deps/miniconda3/bin/activate hssim

# Velocity tracking
# # G1 with FastSAC on IsaacGym
# source scripts/source_isaacsim_setup.sh
# python src/holosoma/holosoma/train_agent.py \
#     exp:g1-29dof-fast-sac \
#     simulator:isaacsim \
#     logger:wandb \
#     --training.seed 1

# Whole-body tracking
# G1 with PPO
source scripts/source_isaacsim_setup.sh
python src/holosoma/holosoma/train_agent.py \
    exp:g1-29dof-wbt \
    logger:wandb