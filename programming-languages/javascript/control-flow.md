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

For-in loops over properties of an object.

```javascript
// loop over elements of an array
var numbers = [45, 4, 9, 16, 25];

var txt = "";
var x;
for (x in numbers) {
  txt += numbers[x] + "<br>";
}
// or:
for (let i in numbers) {
  txt += numbers[i] + "<br>";
}

// loop over properties of an object
var person = {fname:"Bill", lname:"Gates", age:62}; 

var text = "";
var x;
for (x in person) {
    text += person[x]+" "; 
} // text = "Bill Gates 62"
```

## For-of

For-of loops over values of an iterable object.

```javascript
let cars = ["BMW", "Volvo", "Mini"];
let text = "";

for (let x of cars) {
  text += x + "<br>";
}
```

## Reference

\[1\] [https://www.w3schools.com/js/js\_loop\_forin.asp](https://www.w3schools.com/js/js_loop_forin.asp)

