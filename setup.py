from setuptools import setup, find_packages

long_description = """
A fast and scalable task and motion planning framework for tasks expressed in Hierarchical Linear Temporal Logic (H-LTL) and Graph of Convex Set (GCS). 

"""

setup(name="hltlgcs",
        version="0.0.1",
        description="A fast and scalable task and motion planning framework for tasks expressed in Hierarchical Linear Temporal Logic (H-LTL) and Graph of Convex Set (GCS).",
        long_description=long_description,
        url="https://github.com/intelligent-control-lab/Task_Motion_Planning_with_HLTL_and_GCS.git",
        author="Zhongqi Wei, Xusheng Luo",
        author_email="zhongqi2@andrew.cmu.edu",
        license="MIT",
        packages=find_packages(),
        python_requires=">=3.8",
        install_requires=[
            "pydrake",
            "ltlf2dfa",
            "treelib",
            "matplotlib",
            "scipy",
            "sympy",
            "numpy",
            "graphviz",
            "pydot"],
        zip_safe=False)