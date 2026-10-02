"""Run the AmphiBot example: python run_sim.py --experiment_config experiment_config.yaml"""

import os
import sys

# Make the example's controller package importable from any working directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from farms_sim._bootstrap import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
