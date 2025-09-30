from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

with open("requirements.txt", "r", encoding="utf-8") as req_file:
    requirements = req_file.read().splitlines()

setup(
    name="ark_unitree_g1",
    description="Ark Implemention for Unitree G1",
    version="1.0.0",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(),
    license="MIT License",
    install_requires=requirements,
)
