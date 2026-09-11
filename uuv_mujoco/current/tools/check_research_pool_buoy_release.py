"""Compatibility entry point for the force-limited buoy release and rope check."""

import sys
from check_research_pool_magnet_rope import main

if __name__ == "__main__":
    if "--legacy" in sys.argv:
        sys.argv.append("--without_fix")
    main()
