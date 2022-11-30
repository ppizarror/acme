# acme

A small 3D geometry library

## Cloning

To clone this project, the following commands are required in a terminal:

```bash
git clone https://github.com/IE3-CL/acme.git
cd acme
git submodule update --init
```

## Installation

To use the library, it must be created as a submodule in git inside the path of a project:

```bash
git submodule add https://github.com/IE3-CL/acme.git lib/acme
```

This will create the ``acme`` folder inside ``lib``. Then, inside ``CMakeLists.txt``:

```cmake
import_library(lib/acme)
target_link_libraries(MyProject acme)
```

## Documentation

Online documentation is available at: <https://stoccodavide.github.io/acme>. See also
<https://www.sciencedirect.com/science/article/pii/S2352711021001266>.
