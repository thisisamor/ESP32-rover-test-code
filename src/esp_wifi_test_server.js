// this server is used to test the connection between ESP board and server
// when ESP is connected to wifi
// it should be able to send a POST request to the server
// and receive 200 OK as the http code

var express = require('express');
var server = express();
const port = 8080; 

// web content
let htmlContent = `
<!DOCTYPE html>
<html>
 <body>
 <h2>This is a test message</h2>
 <!-- this is a comment: br tag is for line break-->
 </body>
</html>
`;

server.use(express.json());

server.get('/rover/', function(req, res) {
    res.writeHead(200, {'Content-Type':'text/html'});
    res.end(htmlContent);
});  

server.post('/', (req, res) => {
  const postData = req.body;
  // data processing example
  const responseContent = "<p>Received data: " + postData.i + postData.j + "</p>";
  // Send a response back to the client
  res.send(responseContent);
});

server.listen(port, () => {
  console.log(`Server listening on port ${port}`);
}); 
