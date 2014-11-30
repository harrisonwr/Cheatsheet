Firebase Cheatsheet
===============

#### How to make your own textbook exchange

#####Step1: Setup FirebaseRoot
```
.constant('FirebaseRoot', 'https://harrison-testbook.firebaseio.com/');
```

#####Step2: HTML Input Form
Now You will only see the add new button, because we havn't setup angular (partial/home.html)
```html
<!--add new books-->
<div class="row" style='border:1px solid blue;text-align:center;' ng-click="display_input_box()">
<!-- ng-click="..." is to call the function in app.js when click-->
Add A New Book
</div>

<!--Input books-->
<div class="row" style='text-align:center; border:1px solid black; padding:20px;' ng-show="show_input_box">
<!-- ng-show="..." is to determine show the div or not -->
  <div class="col-md-4">
    <label >Name:</label>
    <input type="text" class="form-control" ng-model="name">
    <!-- ng-model="..." is to declare a variable -->
    {{name}}
  </div>
  <div class="col-md-4">
    <label>Book:</label>
    <input type="text" class="form-control" ng-model="book">
    {{book}}
  </div>
  <div class="col-md-4">
    <label>Price:</label>
    <input type="text" class="form-control" ng-model="price">
    {{price}}
  </div>
  <button ng-click="save_book()">save</button>
</div>
```

#####Step3: Make the add button work (app.js)
This will test the functionality of display_input_box()
```javascript
$scope.display_input_box = function(){
  alert('hi');
};
```

#####Step3: Make the add button work (app.js)
Now you should be able to toggle input
```javascript
$scope.display_input_box = function(){
  // change the status of the boolean $scope.show_input_box
  $scope.show_input_box = !$scope.show_input_box;
};
```

#####Step3: Make the save button work firebase (app.js)
Call firebase, and just see it working
```javascript
var fireusers_root = new Firebase(FirebaseRoot);
var fire_textbooks = $firebase(fireusers_root).$asObject();

// show all the data in firebase
$scope.allbooks = fire_textbooks;
```

#####Step4: Save Book (app.js)
See our textbook object // have not save to firebase yet
```javascript
$scope.save_book = function(){
  var textbook = {
    name: $scope.name,
    book: $scope.book,
    price: $scope.price,
  };

  console.log(textbook);

};
```

#####Step4: Now save book work with firebase (app.js)
Change the firebase declaration to asArray
```javascript
var fire_textbooks = $firebase(fireusers_root).$asArray();
```

Update the firebase array by using $add
```javascript
$scope.save_book = function(){
  var textbook = {
    name: $scope.name,
    book: $scope.book,
    price: $scope.price,
  };

  fire_textbooks.$add(textbook).then(function(ref) {
    var id = ref.key();
    $scope.name = "";
    $scope.book = "";
    $scope.price = "";
    });  
  };
```

#####Step5: Display data
(home.html)
```html
<div class="row" ng-repeat="p in allbooks" style='text-align:center;'>
  <div class="col-md-3">{{p.name}}</div>
  <div class="col-md-3">{{p.book}}</div>
  <div class="col-md-3">{{p.price}}</div>
</div>
```

#####Step6: Add remove function
(home.html)
Add button
```html
<div class="row" ng-repeat="p in allbooks" style='text-align:center;'>
  <div class="col-md-3">{{p.name}}</div>
  <div class="col-md-3">{{p.book}}</div>
  <div class="col-md-3">{{p.price}}</div>
  <div class="col-md-3"><button ng-click="remove_book($index)">remove</button></div>
</div>
```

(app.js)
Add functionality
```javascript
$scope.remove_book = function(idx){
  // idx is passed in as the index of the data set
  fire_textbooks.$remove(idx).then(function(ref) {
    var id = ref.key();
    console.log(id);
  });
};
```
