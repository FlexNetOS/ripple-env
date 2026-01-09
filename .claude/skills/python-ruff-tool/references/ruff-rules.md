# Ruff Rules Reference

## Complete Rule Categories

### E - pycodestyle (Errors)

Error-level PEP 8 style violations.

| Rule | Description |
|------|-------------|
| E101 | Indentation contains mixed spaces and tabs |
| E111 | Indentation is not a multiple of four |
| E112 | Expected an indented block |
| E113 | Unexpected indentation |
| E114 | Indentation is not a multiple of four (comment) |
| E115 | Expected an indented block (comment) |
| E116 | Unexpected indentation (comment) |
| E117 | Over-indented |
| E201 | Whitespace after '(' |
| E202 | Whitespace before ')' |
| E203 | Whitespace before ':' |
| E211 | Whitespace before '(' |
| E221 | Multiple spaces before operator |
| E222 | Multiple spaces after operator |
| E223 | Tab before operator |
| E224 | Tab after operator |
| E225 | Missing whitespace around operator |
| E226 | Missing whitespace around arithmetic operator |
| E227 | Missing whitespace around bitwise or shift operator |
| E228 | Missing whitespace around modulo operator |
| E231 | Missing whitespace after ',' |
| E241 | Multiple spaces after ',' |
| E242 | Tab after ',' |
| E251 | Unexpected spaces around keyword / parameter equals |
| E252 | Missing whitespace around parameter equals |
| E261 | At least two spaces before inline comment |
| E262 | Inline comment should start with '# ' |
| E265 | Block comment should start with '# ' |
| E266 | Too many leading '#' for block comment |
| E271 | Multiple spaces after keyword |
| E272 | Multiple spaces before keyword |
| E273 | Tab after keyword |
| E274 | Tab before keyword |
| E275 | Missing whitespace after keyword |
| E301 | Expected 1 blank line, found 0 |
| E302 | Expected 2 blank lines, found N |
| E303 | Too many blank lines |
| E304 | Blank lines found after function decorator |
| E305 | Expected 2 blank lines after class or function definition |
| E306 | Expected 1 blank line before a nested definition |
| E401 | Multiple imports on one line |
| E402 | Module level import not at top of file |
| E501 | Line too long |
| E502 | Backslash is redundant |
| E701 | Multiple statements on one line (colon) |
| E702 | Multiple statements on one line (semicolon) |
| E703 | Statement ends with semicolon |
| E704 | Multiple statements on one line (def) |
| E711 | Comparison to None |
| E712 | Comparison to True/False |
| E713 | Test for membership should be 'not in' |
| E714 | Test for object identity should be 'is not' |
| E721 | Use type() instead of comparing types directly |
| E722 | Bare except |
| E731 | Do not assign a lambda expression |
| E741 | Ambiguous variable name |
| E742 | Ambiguous class definition |
| E743 | Ambiguous function definition |
| E902 | IOError |
| E999 | Syntax error |

### W - pycodestyle (Warnings)

Warning-level PEP 8 style violations.

| Rule | Description |
|------|-------------|
| W191 | Indentation contains tabs |
| W291 | Trailing whitespace |
| W292 | No newline at end of file |
| W293 | Blank line contains whitespace |
| W391 | Blank line at end of file |
| W503 | Line break before binary operator |
| W504 | Line break after binary operator |
| W505 | Doc line too long |
| W605 | Invalid escape sequence |

### F - Pyflakes

Logical errors and undefined names.

| Rule | Description |
|------|-------------|
| F401 | Module imported but unused |
| F402 | Import shadowed by loop variable |
| F403 | 'from module import *' used |
| F404 | Future import not at beginning of file |
| F405 | Name may be undefined from star imports |
| F406 | 'from module import *' only in module level |
| F407 | Future feature not defined |
| F501 | Invalid % format string |
| F502 | % format expected mapping |
| F503 | % format expected sequence |
| F504 | % format unused named arguments |
| F505 | % format missing named arguments |
| F506 | % format mixed positional and named arguments |
| F507 | % format mismatch in argument count |
| F508 | % format with * requires a sequence |
| F509 | % format with unsupported type |
| F521 | .format() invalid format string |
| F522 | .format() unused named arguments |
| F523 | .format() unused positional arguments |
| F524 | .format() missing argument |
| F525 | .format() mixing automatic and manual numbering |
| F601 | Dictionary key literal repeated |
| F602 | Dictionary key variable repeated |
| F621 | Too many expressions in star-unpacking assignment |
| F622 | Multiple starred expressions in assignment |
| F631 | Assert test is a non-empty tuple |
| F632 | Use == to compare with literal |
| F633 | Use of >> is invalid with print function |
| F634 | If test is a tuple, always True |
| F701 | Break outside loop |
| F702 | Continue outside loop |
| F704 | Yield outside function |
| F706 | Return outside function |
| F707 | Default except not last |
| F811 | Redefinition of unused name |
| F821 | Undefined name |
| F822 | Undefined name in __all__ |
| F823 | Local variable referenced before assignment |
| F841 | Local variable assigned but never used |
| F842 | Local variable annotated but never used |
| F901 | Raise NotImplemented instead of NotImplementedError |

### UP - pyupgrade

Modernize Python syntax for newer versions.

| Rule | Description | Fix |
|------|-------------|-----|
| UP001 | Remove unnecessary encoding in open() | Yes |
| UP003 | Use {} instead of type() | Yes |
| UP004 | Remove useless object inheritance | Yes |
| UP005 | Replace deprecated unittest assertions | Yes |
| UP006 | Use type instead of Type for builtins | Yes |
| UP007 | Use X \| Y for union types (3.10+) | Yes |
| UP008 | Use super() without arguments | Yes |
| UP009 | Remove unnecessary UTF-8 encoding declarations | Yes |
| UP010 | Remove unnecessary __future__ imports | Yes |
| UP011 | Remove unnecessary parentheses from functools.lru_cache | Yes |
| UP012 | Remove unnecessary encode("utf-8") | Yes |
| UP013 | Convert TypedDict to class syntax | Yes |
| UP014 | Convert NamedTuple to class syntax | Yes |
| UP015 | Remove unnecessary mode='r' in open() | Yes |
| UP017 | Use datetime.UTC alias (3.11+) | Yes |
| UP018 | Use native str/bytes/int instead of literals | Yes |
| UP019 | Use typing.LiteralString instead of typing_extensions | Yes |
| UP020 | Use builtin open() | Yes |
| UP021 | Replace universal_newlines with text | Yes |
| UP022 | Replace stdout/stderr with capture_output | Yes |
| UP023 | Replace deprecated cElementTree | Yes |
| UP024 | Replace deprecated OSError aliases | Yes |
| UP025 | Remove unicode literal prefix | Yes |
| UP026 | Replace deprecated mock imports | Yes |
| UP027 | Remove unpacked list comprehension | Yes |
| UP028 | Replace yield in for loop with yield from | Yes |
| UP029 | Replace unnecessary builtin imports | Yes |
| UP030 | Use implicit references in format strings | Yes |
| UP031 | Use format specifiers instead of percent format | Yes |
| UP032 | Use f-strings instead of .format() | Yes |
| UP033 | Use @functools.cache instead of @functools.lru_cache(maxsize=None) | Yes |
| UP034 | Remove extraneous parentheses | Yes |
| UP035 | Replace deprecated imports | Yes |
| UP036 | Remove outdated version blocks | Yes |
| UP037 | Remove quotes from type annotations | Yes |
| UP038 | Use X \| Y in isinstance/issubclass | Yes |
| UP039 | Remove unnecessary parentheses in class definition | Yes |
| UP040 | Use type keyword for TypeAlias (3.12+) | Yes |
| UP041 | Replace deprecated timeout_decorator | Yes |
| UP042 | Use PEP 604 union types for isinstance | Yes |

### I - isort

Import sorting and organization.

| Rule | Description | Fix |
|------|-------------|-----|
| I001 | Import block is unsorted or un-formatted | Yes |
| I002 | Missing required import | Yes |

### B - flake8-bugbear

Bug risk patterns and likely mistakes.

| Rule | Description |
|------|-------------|
| B002 | Python does not support increment/decrement operators |
| B003 | Assigning to os.environ does not clear |
| B004 | Using hasattr with __call__ |
| B005 | Using .strip() with multi-character strings |
| B006 | Mutable default argument |
| B007 | Loop control variable not used |
| B008 | Function call in default argument |
| B009 | Getattr with constant attribute value |
| B010 | Setattr with constant attribute value |
| B011 | Assert False should be debugger |
| B012 | Return in finally block |
| B013 | Redundant tuple in exception handler |
| B014 | Redundant exception types |
| B015 | Pointless comparison |
| B016 | Raise literal |
| B017 | assertRaises with no match |
| B018 | Useless expression |
| B019 | @lru_cache on method |
| B020 | Loop control variable overrides iterable |
| B021 | f-string docstrings |
| B022 | Useless contextlib.suppress |
| B023 | Function uses loop variable |
| B024 | Abstract base class without abstract methods |
| B025 | Duplicate try-except handlers |
| B026 | Star-arg unpacking after keyword argument |
| B027 | Empty method in abstract base class |
| B028 | No stacklevel in warnings.warn |
| B029 | except with empty tuple |
| B030 | except with duplicate handlers |
| B031 | Groupby with non-lazy call |
| B032 | Possible unintentional type annotation |
| B033 | Duplicate set element |
| B034 | Re.sub with count without flags |
| B035 | Dictionary comprehension with static key |
| B904 | Raise without from in except handler |
| B905 | Zip without explicit strict parameter |

### C4 - flake8-comprehensions

Comprehension improvements and simplifications.

| Rule | Description | Fix |
|------|-------------|-----|
| C400 | Unnecessary generator (use list comprehension) | Yes |
| C401 | Unnecessary generator (use set comprehension) | Yes |
| C402 | Unnecessary generator (use dict comprehension) | Yes |
| C403 | Unnecessary list comprehension (use set) | Yes |
| C404 | Unnecessary list comprehension (use dict) | Yes |
| C405 | Unnecessary literal (use set literal) | Yes |
| C406 | Unnecessary literal (use dict literal) | Yes |
| C408 | Unnecessary dict/list/tuple call | Yes |
| C409 | Unnecessary list passed to tuple() | Yes |
| C410 | Unnecessary list passed to list() | Yes |
| C411 | Unnecessary list call (use list literal) | Yes |
| C413 | Unnecessary call around sorted() | Yes |
| C414 | Unnecessary double call | Yes |
| C415 | Unnecessary subscript reversal | Yes |
| C416 | Unnecessary comprehension (use list/set/dict) | Yes |
| C417 | Unnecessary map (use generator/comprehension) | Yes |
| C418 | Unnecessary dict call in dict() | Yes |
| C419 | Unnecessary comprehension in any/all | Yes |

### SIM - flake8-simplify

Code simplification suggestions.

| Rule | Description | Fix |
|------|-------------|-----|
| SIM101 | Multiple isinstance calls (combine) | Yes |
| SIM102 | Nested if statements (combine) | Yes |
| SIM103 | Return boolean condition directly | Yes |
| SIM105 | Use contextlib.suppress() | Yes |
| SIM107 | Return in try/except/finally | No |
| SIM108 | Use ternary operator | Yes |
| SIM109 | Compare with multiple values using in | Yes |
| SIM110 | Use any() | Yes |
| SIM111 | Use all() | Yes |
| SIM112 | Use os.environ.get() | Yes |
| SIM113 | Use enumerate() | Yes |
| SIM114 | Combine if branches using or | Yes |
| SIM115 | Use context manager for opening files | No |
| SIM116 | Use dictionary instead of if-elif | No |
| SIM117 | Combine with statements | Yes |
| SIM118 | Use key in dict | Yes |
| SIM201 | Use not (a == b) | Yes |
| SIM202 | Use not (a != b) | Yes |
| SIM208 | Use not (not a) | Yes |
| SIM210 | Use bool() | Yes |
| SIM211 | Use not bool() | Yes |
| SIM212 | Use a if a else b | Yes |
| SIM220 | Use False (a and not a) | Yes |
| SIM221 | Use True (a or not a) | Yes |
| SIM222 | Use True (or True) | Yes |
| SIM223 | Use False (and False) | Yes |
| SIM300 | Yoda condition | Yes |
| SIM401 | Use dict.get() | Yes |
| SIM904 | Initialize dict with dict literal | Yes |
| SIM905 | Split static string | Yes |
| SIM910 | Use dict.get() with default None | Yes |
| SIM911 | Use zip() instead of dict items | Yes |

### RUF - Ruff-specific

Ruff's own custom rules.

| Rule | Description | Fix |
|------|-------------|-----|
| RUF001 | Ambiguous unicode character in string | Yes |
| RUF002 | Ambiguous unicode character in docstring | Yes |
| RUF003 | Ambiguous unicode character in comment | Yes |
| RUF005 | Collection literal concatenation | Yes |
| RUF006 | Asyncio dangling task | No |
| RUF007 | Pairwise over zipped | Yes |
| RUF008 | Mutable class default in dataclass | No |
| RUF009 | Function call in dataclass default | No |
| RUF010 | Use explicit conversion flag | Yes |
| RUF011 | Static key in dict comprehension | No |
| RUF012 | Mutable class attribute annotation | No |
| RUF013 | Implicit Optional | Yes |
| RUF015 | Prefer next(iter()) | Yes |
| RUF016 | Invalid index type | No |
| RUF017 | Quadratic list summation | Yes |
| RUF018 | Avoid assignment expressions in assert | No |
| RUF019 | Unnecessary key check | Yes |
| RUF100 | Unused noqa directive | Yes |
| RUF200 | Invalid pyproject.toml | No |

## Recommended Rule Sets

### Minimal (Safe defaults)

```toml
[tool.ruff.lint]
select = ["E", "F"]
```

### Recommended (Good balance)

```toml
[tool.ruff.lint]
select = ["E", "F", "W", "UP", "I", "B", "C4"]
```

### Comprehensive (Full modernization)

```toml
[tool.ruff.lint]
select = ["E", "F", "W", "UP", "I", "B", "C4", "SIM", "RUF"]
```

### Python Modernization Only

```toml
[tool.ruff.lint]
select = ["UP"]
```
