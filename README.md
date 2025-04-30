# Task_Motion_Planning_with_HLTL_and_GCS

A fast and scalable task and motion planning framework for tasks expressed in Hierarchical Linear Temporal Logic (H-LTL). 

This repository contains code to accompany the paper [*Hierarchical Temporal Logic Task and Motion Planning for Multi-Robot Systems*](https://arxiv.org/abs/2504.18899) by Zhongqi Wei, Xusheng Luo and Changliu Liu. 

## Installation

Make sure all dependencies are installed, then:

```
$ git clone https://github.com/vincekurtz/ltl_gcs
$ cd ltl_gcs
$ pip install .
```

## Dependencies

- [Drake](https://drake.mit.edu/)
- [MONA](https://www.brics.dk/mona/download.html)
- [ltlf2dfa](https://github.com/whitemech/LTLf2DFA)
- [MOSEK](https://www.mosek.com/) (license only)
- treelib
- matplotlib
- scipy
- sympy
- numpy
- graphviz
- pydot

Of these, only MONA and MOSEK require special consideration: all others can be
installed with `pip`. For MOSEK, you only need a valid license: MOSEK itself is
installed along with Drake. 

## Examples

The following examples and several other can be found in the `examples`
directory.
