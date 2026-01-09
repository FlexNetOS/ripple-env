# F-String Patterns Reference

## Complete Conversion Patterns

### Basic Printf-style Conversions

#### Simple String Substitution

```python
# Input
"Hello, %s" % name
"Hello, %s!" % (name,)

# Output
f"Hello, {name}"
f"Hello, {name}!"
```

#### Multiple Values

```python
# Input
"%s %s" % (first, last)
"%s, %s, and %s" % (a, b, c)

# Output
f"{first} {last}"
f"{a}, {b}, and {c}"
```

#### Named Placeholders

```python
# Input
"%(name)s is %(age)d years old" % {"name": name, "age": age}
"%(key)s: %(value)s" % {"key": k, "value": v}

# Output
f"{name} is {age} years old"
f"{k}: {v}"
```

### Format Specifiers

#### Integer Formatting

```python
# Input
"Count: %d" % count
"Hex: %x" % value
"Hex upper: %X" % value
"Octal: %o" % value
"Padded: %05d" % num

# Output
f"Count: {count}"
f"Hex: {value:x}"
f"Hex upper: {value:X}"
f"Octal: {value:o}"
f"Padded: {num:05d}"
```

#### Float Formatting

```python
# Input
"Price: %.2f" % price
"Scientific: %e" % value
"General: %g" % value
"Width: %10.2f" % num

# Output
f"Price: {price:.2f}"
f"Scientific: {value:e}"
f"General: {value:g}"
f"Width: {num:10.2f}"
```

#### String Width and Alignment

```python
# Input
"%-20s" % text      # Left-aligned
"%20s" % text       # Right-aligned

# Output
f"{text:<20}"       # Left-aligned
f"{text:>20}"       # Right-aligned
```

### .format() Conversions

#### Positional Arguments

```python
# Input
"{}".format(value)
"{0}".format(value)
"{0} {1}".format(first, last)
"{1} {0}".format(first, last)  # Reversed order

# Output
f"{value}"
f"{value}"
f"{first} {last}"
f"{last} {first}"
```

#### Keyword Arguments

```python
# Input
"{name}".format(name=name)
"{name} is {age}".format(name=n, age=a)

# Output
f"{name}"
f"{n} is {a}"
```

#### Mixed Arguments

```python
# Input
"{0} {name}".format(greeting, name=person)

# Output
f"{greeting} {person}"
```

#### Format Specs

```python
# Input
"{:>10}".format(text)
"{:^20}".format(centered)
"{:_<10}".format(padded)
"{:.2f}".format(price)
"{:,}".format(large_number)
"{:+.2f}".format(change)

# Output
f"{text:>10}"
f"{centered:^20}"
f"{padded:_<10}"
f"{price:.2f}"
f"{large_number:,}"
f"{change:+.2f}"
```

### String Concatenation (--transform-concats)

#### Simple Concatenation

```python
# Input
"Hello, " + name
name + " World"
"Hello, " + name + "!"

# Output
f"Hello, {name}"
f"{name} World"
f"Hello, {name}!"
```

#### With str() Conversion

```python
# Input
"Count: " + str(count)
"Value: " + str(value) + " units"

# Output
f"Count: {count}"
f"Value: {value} units"
```

#### Multiple Variables

```python
# Input
first + " " + middle + " " + last
a + b + c + d

# Output
f"{first} {middle} {last}"
f"{a}{b}{c}{d}"
```

### Static Joins (--transform-joins)

#### Simple Static Lists

```python
# Input
", ".join(["a", "b", "c"])
" ".join(["Hello", "World"])
"-".join(["2024", "01", "15"])

# Output
"a, b, c"
"Hello World"
"2024-01-15"
```

#### Variable Joins

```python
# Input
" ".join([first, last])
", ".join([item1, item2, item3])

# Output
f"{first} {last}"
f"{item1}, {item2}, {item3}"
```

### Complex Patterns

#### Nested Expressions

```python
# Input
"Result: {}".format(calculate(x))
"Sum: {}".format(a + b)
"List: {}".format([x for x in items])

# Output
f"Result: {calculate(x)}"
f"Sum: {a + b}"
f"List: {[x for x in items]}"
```

#### Attribute Access

```python
# Input
"Name: {}".format(obj.name)
"{}.{}".format(module, function)

# Output
f"Name: {obj.name}"
f"{module}.{function}"
```

#### Dictionary Access

```python
# Input
"Value: {}".format(data["key"])
"{}: {}".format(key, data[key])

# Output
f"Value: {data['key']}"
f"{key}: {data[key]}"
```

#### Method Calls

```python
# Input
"Upper: {}".format(text.upper())
"Stripped: {}".format(s.strip())

# Output
f"Upper: {text.upper()}"
f"Stripped: {s.strip()}"
```

### Multiline Strings

#### Simple Multiline

```python
# Input
"""Name: {}
Age: {}
City: {}""".format(name, age, city)

# Output
f"""Name: {name}
Age: {age}
City: {city}"""
```

#### With Line Continuation

```python
# Input
("Hello, "
 "{}"
 "!").format(name)

# Output
f"Hello, {name}!"
```

## Patterns Flynt Skips

### Too Complex Expressions

```python
# Skipped: expression too long
"result: {}".format(
    very_long_function_name_with_many_parameters(
        parameter_one,
        parameter_two,
        parameter_three
    )
)
```

### Would Make Code Longer

```python
# Skipped: f-string would be longer
"{}".format(x) if condition else ""
```

### Unsafe Conversions

```python
# Skipped: tuple display difference
'%s' % (1,)  # prints "1"
# f'{(1,)}' would print "(1,)"
```

### Dynamic Format Strings

```python
# Skipped: format string is a variable
format_string.format(value)
"{:{}}".format(value, width)  # Dynamic width
```

## Format Spec Quick Reference

| Spec | Meaning | Example |
|------|---------|---------|
| `<` | Left align | `f"{x:<10}"` |
| `>` | Right align | `f"{x:>10}"` |
| `^` | Center | `f"{x:^10}"` |
| `+` | Show sign | `f"{x:+}"` |
| `-` | Negative only (default) | `f"{x:-}"` |
| ` ` | Space for positive | `f"{x: }"` |
| `,` | Thousands separator | `f"{x:,}"` |
| `_` | Thousands separator (alt) | `f"{x:_}"` |
| `.n` | Precision | `f"{x:.2f}"` |
| `0n` | Zero-padded | `f"{x:05d}"` |
| `b` | Binary | `f"{x:b}"` |
| `o` | Octal | `f"{x:o}"` |
| `x` | Hex lowercase | `f"{x:x}"` |
| `X` | Hex uppercase | `f"{x:X}"` |
| `e` | Scientific | `f"{x:e}"` |
| `f` | Fixed-point | `f"{x:f}"` |
| `%` | Percentage | `f"{x:.1%}"` |

## Special Characters in F-Strings

### Escaping Braces

```python
# To include literal braces
f"{{value}}"  # prints "{value}"
f"JSON: {{{data}}}"  # prints "JSON: {actual_data}"
```

### Backslashes

```python
# Backslashes not allowed in expressions
# This doesn't work:
# f"path: {path.replace('\\', '/')}"

# Solution: use a variable
replacement = path.replace('\\', '/')
f"path: {replacement}"
```

### Quotes

```python
# Alternate quote types
f"He said '{name}'"
f'She said "{name}"'
f"""Multi-line with "quotes" and 'apostrophes'"""
```

## Performance Comparison

```python
# Benchmark results (relative speed)
# f-strings:      1.0x (fastest)
# .format():      1.6x slower
# %-formatting:   1.3x slower
# concatenation:  varies (can be slower for many items)
```

F-strings are not just more readable but also faster because they're evaluated at compile time rather than runtime.
