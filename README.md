# IARRC-2017
[![Build Status](https://travis-ci.org/UBC-Snowbots/IARRC-2017.svg?branch=master)](https://travis-ci.org/UBC-Snowbots/IARRC-2017)

UBC Snowbots Repo for the 2017 IARRC Competition

Run `get_started.sh` if you don't have a development environment.

It's the same to the one on the [Snowbots/IGVC-2017](https://github.com/UBC-Snowbots/IGVC-2017) repository.

## Misc
- see devices on the network with `arp -a` (install arp if you don't have it)
- ssh to the NUC with `ssh sb@nuc02`
- run `export ROS_MASTER_URI=http://nuc02:11311` in a terminal, and any ros commands run from that terminal after will connect to the nuc (for example, `rostopic list`)
