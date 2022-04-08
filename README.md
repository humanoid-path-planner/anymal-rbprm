# anymal-rbprm

[![Pipeline status](https://gitlab.laas.fr/humanoid-path-planner/anymal-rbprm/badges/master/pipeline.svg)](https://gitlab.laas.fr/humanoid-path-planner/anymal-rbprm/commits/master)
[![Coverage report](https://gitlab.laas.fr/humanoid-path-planner/anymal-rbprm/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/humanoid-path-planner/anymal-rbprm/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/humanoid-path-planner/anymal-rbprm/master.svg)](https://results.pre-commit.ci/latest/github/humanoid-path-planner/anymal-rbprm)

File database for anymal robot using the hpp-rbprm framework

## Installation instruction

```bash
mkdir build ; cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DPYTHON_EXECUTABLE=$(which python{X}) -DCMAKE_INSTALL_PREFIX={INSTALL_PATH} ..
make install
```

Replace `{X}` with your Python version (2 or 3) and `{INSTALL_PATH}` with the desired path.
