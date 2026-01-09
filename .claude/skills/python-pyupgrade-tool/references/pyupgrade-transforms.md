# Pyupgrade Transformations Reference

## Complete List of Transformations by Category

### String Formatting

#### Printf-style to f-strings

```python
# Input
"Hello, %s" % name
"Hello, %s %s" % (first, last)
"Value: %d" % (count,)
"%.2f" % price

# Output (--py36-plus)
f"Hello, {name}"
f"Hello, {first} {last}"
f"Value: {count}"
f"{price:.2f}"
```

#### .format() to f-strings

```python
# Input
"Hello, {}".format(name)
"Hello, {0}".format(name)
"Hello, {name}".format(name=name)
"{} + {} = {}".format(a, b, a + b)

# Output (--py36-plus)
f"Hello, {name}"
f"Hello, {name}"
f"Hello, {name}"
f"{a} + {b} = {a + b}"
```

#### Format Spec Preservation

```python
# Input
"{:>10}".format(text)
"{:.2f}".format(value)
"{:,}".format(number)

# Output
f"{text:>10}"
f"{value:.2f}"
f"{number:,}"
```

### Type Annotations

#### Optional to Union with None (--py310-plus)

```python
# Input
from typing import Optional
def func(x: Optional[int]) -> Optional[str]:
    pass

# Output
def func(x: int | None) -> str | None:
    pass
```

#### Union Types (--py310-plus)

```python
# Input
from typing import Union
def func(x: Union[int, str]) -> Union[bool, None]:
    pass

# Output
def func(x: int | str) -> bool | None:
    pass
```

#### Generic Types (--py39-plus)

```python
# Input
from typing import List, Dict, Set, Tuple, FrozenSet

def func(
    items: List[int],
    mapping: Dict[str, int],
    unique: Set[str],
    pair: Tuple[int, str],
    frozen: FrozenSet[int],
) -> List[str]:
    pass

# Output
def func(
    items: list[int],
    mapping: dict[str, int],
    unique: set[str],
    pair: tuple[int, str],
    frozen: frozenset[int],
) -> list[str]:
    pass
```

#### Type from typing module (--py39-plus)

```python
# Input
from typing import Type
def func(cls: Type[MyClass]) -> Type[MyClass]:
    pass

# Output
def func(cls: type[MyClass]) -> type[MyClass]:
    pass
```

### Class Definitions

#### Remove object Inheritance

```python
# Input
class MyClass(object):
    pass

class AnotherClass(object, Mixin):
    pass

# Output
class MyClass:
    pass

class AnotherClass(Mixin):
    pass
```

#### Remove __metaclass__

```python
# Input
class MyClass(object):
    __metaclass__ = ABCMeta

# Output
class MyClass(metaclass=ABCMeta):
    pass
```

#### Modern super() Calls

```python
# Input
class Child(Parent):
    def __init__(self):
        super(Child, self).__init__()

    def method(self):
        return super(Child, self).method()

# Output
class Child(Parent):
    def __init__(self):
        super().__init__()

    def method(self):
        return super().method()
```

### Import Updates

#### collections.abc Migration

```python
# Input
from collections import Mapping, MutableMapping, Sequence
from collections import OrderedDict, defaultdict

# Output
from collections.abc import Mapping, MutableMapping, Sequence
from collections import OrderedDict, defaultdict  # These stay
```

#### Mock Import Update

```python
# Input
from mock import Mock, patch, MagicMock
import mock

# Output (unless --keep-mock)
from unittest.mock import Mock, patch, MagicMock
from unittest import mock
```

#### __future__ Import Cleanup

```python
# Input (targeting Python 3.10+)
from __future__ import (
    absolute_import,
    division,
    print_function,
    unicode_literals,
    annotations,
)

# Output
from __future__ import annotations  # Only kept if still needed
```

#### Deprecated Module Replacements

```python
# Input
from xml.etree.cElementTree import Element
import cPickle

# Output
from xml.etree.ElementTree import Element
import pickle
```

### Built-in Updates

#### Set Literals

```python
# Input
set([1, 2, 3])
set((1, 2, 3))
set([x for x in items])

# Output
{1, 2, 3}
{1, 2, 3}
{x for x in items}
```

#### Dict Comprehensions

```python
# Input
dict([(k, v) for k, v in items])
dict((k, v) for k, v in items)

# Output
{k: v for k, v in items}
{k: v for k, v in items}
```

#### open() Simplification

```python
# Input
open("file.txt", "r")
open("file.txt", mode="r")
io.open("file.txt")

# Output
open("file.txt")
open("file.txt")
open("file.txt")
```

#### Encoding in open()

```python
# Input
open("file.txt", encoding="utf-8")  # Python 3.15+ will change default

# Stays the same (explicit encoding is good practice)
open("file.txt", encoding="utf-8")
```

### Exception Handling

#### OSError Aliases

```python
# Input
except IOError:
    pass
except EnvironmentError:
    pass
except WindowsError:  # Windows only
    pass
except socket.error:
    pass
except select.error:
    pass
except mmap.error:
    pass

# Output
except OSError:
    pass
except OSError:
    pass
except OSError:
    pass
except OSError:
    pass
except OSError:
    pass
except OSError:
    pass
```

#### Timeout Exception

```python
# Input (--py311-plus)
except asyncio.TimeoutError:
    pass

# Output
except TimeoutError:
    pass
```

### Six Library Removal

```python
# Input
import six

if six.PY2:
    string_types = basestring
else:
    string_types = str

text = six.text_type(value)
binary = six.binary_type(value)
six.moves.range(10)
six.moves.urllib.parse.quote(s)

# Output
string_types = str

text = str(value)
binary = bytes(value)
range(10)
urllib.parse.quote(s)
```

### Escape Sequences

```python
# Input
regex = "\d+\s+"
path = "C:\Users\name"

# Output
regex = r"\d+\s+"
path = r"C:\Users\name"  # or "C:\\Users\\name"
```

### Comparison Operators

```python
# Input
if x is 1:
    pass
if y is "string":
    pass
if z is True:
    pass

# Output
if x == 1:
    pass
if y == "string":
    pass
if z is True:  # This one stays (boolean comparison)
    pass
```

### functools Updates

#### lru_cache Simplification (--py38-plus)

```python
# Input
from functools import lru_cache

@lru_cache()
def func():
    pass

@lru_cache(maxsize=None)
def func2():
    pass

# Output
from functools import lru_cache, cache

@lru_cache
def func():
    pass

@cache  # --py39-plus
def func2():
    pass
```

### isinstance/issubclass Tuple Simplification

```python
# Input (--py310-plus)
isinstance(x, (int, str))
issubclass(cls, (Base1, Base2))

# Output
isinstance(x, int | str)
issubclass(cls, Base1 | Base2)
```

### Named Expressions (Walrus Operator Context)

Pyupgrade doesn't add walrus operators but works correctly with code that uses them:

```python
# Input
values = [y for x in items if (y := process(x))]

# Stays the same (already modern)
values = [y for x in items if (y := process(x))]
```

## Version-Specific Summary

| Upgrade | --py36+ | --py37+ | --py38+ | --py39+ | --py310+ | --py311+ |
|---------|---------|---------|---------|---------|----------|----------|
| f-strings | X | X | X | X | X | X |
| Remove object inheritance | X | X | X | X | X | X |
| super() without args | X | X | X | X | X | X |
| collections.abc | X | X | X | X | X | X |
| mock -> unittest.mock | X | X | X | X | X | X |
| @lru_cache without () | | | X | X | X | X |
| @cache decorator | | | | X | X | X |
| list[int] generics | | | | X | X | X |
| dict \| dict merging | | | | X | X | X |
| X \| Y union types | | | | | X | X |
| match statements | | | | | X | X |
| TimeoutError builtin | | | | | | X |
