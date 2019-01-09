from setuptools import setup

setup(
    name="frankapanda",
    version="1.0.0",
    description="Python bindings for Franka Panda dynamics.",
    license="MIT",
    author="Toki Migimatsu",
    packages=["frankapanda"],
    install_requires=["numpy"]
)
