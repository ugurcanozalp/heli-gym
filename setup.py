from setuptools import setup, find_packages

setup(name='heligym',
    version='0.0.1',
    description='A GYM environment package for reinforcement learning' \
      'for helicopter flight tasks using minimum complexity helicopter model.',
    url='https://github.com/Gor-Ren/gym-jsbsim',
    author='Uğurcan Özalp, Gökçay Kabataş',
    license='MIT',
    classifiers=[
      'License :: OSI Approved :: MIT License',
      'Development Status :: 2 - Pre-Alpha',
      'Intended Audience :: Science/Research',
      'Programming Language :: Python :: 3.8',
      'Topic :: Aerospace :: Artificial Intelligence :: Reinforcement Learning',
    ],
    install_requires=['numpy', 'gym', 'pyyaml'],
    python_requires='>=3.8',
    packages = find_packages(),
    include_package_data=True,
    zip_safe=False
    )