# Type & Reflection

```python
# get class info of an object
type(object)
a = 1
type(a) == int # True
type(a) == float # False

# Return True if the object argument is an instance of the classinfo argument, 
# or of a (direct, indirect or virtual) subclass thereof. 
# If object is not an object of the given type, the function always returns False. 
# If classinfo is a tuple of type objects (or recursively, other such tuples), 
# return True if object is an instance of any of the types. 
# If classinfo is not a type or tuple of types and such tuples, 
# a TypeError exception is raised.
isinstance(object, classinfo)
```

