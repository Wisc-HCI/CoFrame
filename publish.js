var ghpages = require('gh-pages');

ghpages.publish('dist', {push: false}, function(err) {console.log(err)});