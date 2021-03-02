# Control Flow

## If

```javascript
if (cond1) {
    // ...
} else if (cond2) {
    // ...
} else {
    // ...
}
```

## Switch

```javascript
switch (x) {
    case 0: // switch-case uses a === match
        // ...
        break;
    case 1:
    case 2: // in JS, the code block can be shared between cases like C
        // ...
        break;
    // ...
    default:
        // ...
}
```

## For

```javascript
for (i = 0; i < cars.length; i++) { 
    text += cars[i] + "<br>";
}
```

## For-in

```javascript
var person = {fname:"Bill", lname:"Gates", age:62}; 

var text = "";
var x;
for (x in person) {
    text += person[x]+" "; 
} // text = "Bill Gates 62"
```

