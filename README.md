# anymal-rbprm

[![Pipeline status](https://gitlab.laas.fr/humanoid-path-planner/anymal-rbprm/badges/master/pipeline.svg)](https://gitlab.laas.fr/humanoid-path-planner/anymal-rbprm/commits/master)
[![Coverage report](https://gitlab.laas.fr/humanoid-path-planner/anymal-rbprm/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/humanoid-path-planner/anymal-rbprm/master/coverage/)

File database for anymal robot using the hpp-rbprm framework

## Installation instruction

```bash
mkdir build ; cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DPYTHON_EXECUTABLE=$(which python{X}) -DCMAKE_INSTALL_PREFIX={INSTALL_PATH} ..
make install
```

Replace `{X}` with your Python version (2 or 3) and `{INSTALL_PATH}` with the desired path.
