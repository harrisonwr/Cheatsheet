Ionic Sidemenu Cheatsheet
=========================

### How to Started Ionic Sidemenu Project

####In terminal
#####Getting Started
```
$ ionic start myApp sidemenu
```
#####Test it on browser
```
$ ionic serve
```
#####Test it on IOS simulator
```
$ cd myApp
$ ionic platform add ios
$ ionic bulid ios
$ ionic emulate ios
```

### What Each File Do
####./www/js
```
app.js
--- Declare where each button linked to
controller.js
--- Declare all the controllers and functions in each controller

```

####./www/templates
```
browse.html
--- "Sidemenu -> Browse"
--------------
login.html
--- "Sidemenu -> Login"
--------------
Menu.html
--- "Sidemenu"
--------------
Playlist.html
--- "Sidemenu -> Playlists -> 'Rap ... etc.'"
--------------
Playlists.html
--- "Sidemenu -> Playlists"
--------------
Search.html
--- "Sidemenu -> Search"

*** Each html file related to one page
```

#### app.js
```javascript
///////////////////////
// link to side menu
// No need to change this section, but change "menu.html"
///////////////////////
  .state('app', {
  url: "/app",
  abstract: true,
  templateUrl: "templates/menu.html",
  controller: 'AppCtrl'
  })

//////////////////////
// link to search.html
// put your own page to replace this part (Ex. home.html)
// browse.html, playlists.html are the same structure
///////////////////////
  .state('app.search', {
    url: "/search",
    views: {
      'menuContent': {
        templateUrl: "templates/search.html"
      }
    }
  })

///////////////////////
// link to playlist.html
// This is the example for passing in id to get specific information
///////////////////////
  .state('app.single', {
    url: "/playlists/:playlistId",
    views: {
      'menuContent': {
        templateUrl: "templates/playlist.html",
        controller: 'PlaylistCtrl'
      }
    }
  });

```
####How to create a new page
#####Step One - Create a new file "pageName.html" in templates
```html
<!-- Nav - Search -->
<ion-view view-title="new page's title">
  <ion-content>
    <h1>New Titles</h1>
  </ion-content>
</ion-view>
```
#####Step Two - Add new .state in app.js
```javascript
.state('app.pageName', {
  url: "/pageName",
  views: {
    'menuContent': {
      templateUrl: "templates/pageName.html"
    }
  }
})

```
#####Step Three - Add new item in menu.html
```html
<ion-side-menu side="left">
  <ion-header-bar class="bar-stable">
    <h1 class="title">Left</h1>
  </ion-header-bar>
  <ion-content>
    <ion-list>
      <ion-item nav-clear menu-close ng-click="login()">
        Login
      </ion-item>

      /*****************************/
      /*New Page menu item add here*/
      /*****************************/
      <ion-item nav-clear menu-close href="#/app/home">
        New Page's Name
      </ion-item>

      <ion-item nav-clear menu-close href="#/app/search">
        Search
      </ion-item>
      <ion-item nav-clear menu-close href="#/app/browse">
        Browse
      </ion-item>
      <ion-item nav-clear menu-close href="#/app/playlists">
        Playlists
      </ion-item>
    </ion-list>
  </ion-content>
  </ion-side-menu>
</ion-side-menus>

```
####How to link to firebase
#####Step One - import firebase and fireangular into index.html
``` html
// index.html
<script src="https://cdn.firebase.com/js/client/2.0.4/firebase.js"></script>
<script src="https://cdn.firebase.com/libs/angularfire/0.9.0/angularfire.min.js"></script>

```
#####Step Two - create a variable that link to firebase
``` javascript
// app.js

//Add "firebase" at the module
angular.module('starter', ['ionic', 'starter.controllers', 'firebase'])
// Add this at the end of app.js
// Remember to delete ; for previous line
.constant('FirebaseRoot', 'https://textbooktest.firebaseio.com/');

```

#####Step Three - import services and
``` javascript
// controller.js
.controller("homeController", ['$scope', '$firebase', 'FirebaseRoot', '$ionicBackdrop', '$timeout', function($scope, $firebase, FirebaseRoot, $ionicBackdrop, $timeout){
  var fireusers_root = new Firebase(FirebaseRoot);
  var fire_textbooks = $firebase(fireusers_root).$asArray();
  // show all the data in firebase
  $scope.allbooks = fire_textbooks;
}]);

```

####How to upload project onto phone
```
1. $ ionic platform add ios
2. $ ionic build ios
3. $ ionic run ios
```
####How to add plugin to enable Facebook login(Firebase need this)
```
1. $ ionic plugin add org.apache.cordova.inappbrowser
```
