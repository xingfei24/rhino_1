import express from 'express';
import { createServer } from 'http';
import { Server } from 'socket.io';
import { QuadrupedRobot } from './robot/QuadrupedRobot.js';

const app = express();
const server = createServer(app);
const io = new Server(server);

// Create robot instance
const robot = new QuadrupedRobot();

// Serve static files
app.use(express.static('public'));

// WebSocket connection handling
io.on('connection', (socket) => {
  console.log('Client connected');

  // Handle robot commands
  socket.on('walk', (data) => {
    const { cycles, velocity, direction } = data;
    robot.walk(cycles, velocity, direction);
    socket.emit('status', robot.getStatus());
  });

  socket.on('stand', () => {
    robot.stand();
    socket.emit('status', robot.getStatus());
  });

  socket.on('balance', () => {
    robot.balance();
    socket.emit('status', robot.getStatus());
  });

  socket.on('disconnect', () => {
    console.log('Client disconnected');
  });
});

const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});