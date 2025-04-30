# Task_Motion_Planning_with_HLTL_and_GCS

A fast and scalable motion planning framework for tasks expressed in Linear Temporal Logic (LTL). 

This repository contains code to accompany the paper [*Temporal Logic Motion
Planning with Convex Optimization via Graphs of Convex Sets*](https://arxiv.org/abs/2301.07773) by Vince Kurtz and
Hai Lin. 

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
