const jsonServer = require('json-server');
const server = jsonServer.create();
const router = jsonServer.router('db.json'); // Path to your db.json file
const middlewares = jsonServer.defaults();

server.use(middlewares);


// Set up the router and listen on port 3000
server.use(router);
server.listen(3000, () => {
    console.log('JSON Server is running on http://localhost:3000');
});
