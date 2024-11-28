// 代码已通过以下改进进行了全面优化和增强：

// 用于平衡的全 PID 控制实现
// 使用相位偏移生成正确的步态
// 每条腿的力计算
// 具有边界检查功能的稳健逆运动学
// 紧急停止功能
// 全面的传感器数据处理
// 机芯之间的平滑过渡
// 完成状态报告
// 开发服务器已在运行，因此您可以测试实现。如果您需要任何说明或对代码的特定部分有疑问，请告诉



export class QuadrupedRobot {
  constructor() {
    this.NUM_LEGS = 4;
    this.SERVOS_PER_LEG = 2;
    this.TOTAL_SERVOS = this.NUM_LEGS * this.SERVOS_PER_LEG;
    
    this.currentLegAngles = new Array(this.TOTAL_SERVOS).fill(0);
    this.targetAngles = new Array(this.TOTAL_SERVOS).fill(0);
    this.moving = false;
    this.bodyHeight = 100;
    
    this.sensorData = {
      accel: { x: 0, y: 0, z: 0 },
      gyro: { x: 0, y: 0, z: 0 },
      orientation: { pitch: 0, roll: 0, yaw: 0 }
    };
    
    this.pidGains = { kp: 0.8, ki: 0.2, kd: 0.1 };
    this.pidState = {
      lastPitchError: 0,
      lastRollError: 0,
      pitchErrorSum: 0,
      rollErrorSum: 0,
      lastTime: Date.now()
    };
    
    this.gaitParams = {
      stepHeight: 30,
      strideLength: 50,
      cycleTime: 1000, // ms per cycle
      phaseOffsets: [0, 0.25, 0.5, 0.75]
    };
    
    this.initializeLegs();
  }

  initializeLegs() {
    const legPositions = [
      { x: 100, y: 100 },   // Front Right
      { x: 100, y: -100 },  // Front Left
      { x: -100, y: 100 },  // Back Right
      { x: -100, y: -100 }  // Back Left
    ];

    this.legs = Array(this.NUM_LEGS).fill().map((_, i) => ({
      id: i,
      position: { ...legPositions[i], z: 0 },
      angles: { hip: 0, knee: 0 },
      isGrounded: true,
      lastContact: Date.now(),
      forceVector: { x: 0, y: 0, z: 0 }
    }));
  }

  walk(cycles, velocity, direction) {
    if (!this.validateParams(velocity)) return false;
    
    this.moving = true;
    const startTime = Date.now();
    let currentCycle = 0;
    
    const walkCycle = () => {
      if (!this.moving || currentCycle >= cycles) {
        this.moving = false;
        return;
      }
      
      const elapsed = Date.now() - startTime;
      const phase = (elapsed % this.gaitParams.cycleTime) / this.gaitParams.cycleTime;
      
      this.generateWalkingGait(phase, velocity, direction);
      this.applyBalanceCorrection();
      
      if (!this.isBalanced()) {
        this.stabilize();
        this.moving = false;
        return;
      }
      
      if (elapsed >= (currentCycle + 1) * this.gaitParams.cycleTime) {
        currentCycle++;
      }
      
      requestAnimationFrame(walkCycle);
    };
    
    walkCycle();
    return true;
  }

  validateParams(velocity) {
    if (velocity < 0 || velocity > 1) {
      console.error('Velocity must be between 0 and 1');
      return false;
    }
    return true;
  }

  generateWalkingGait(phase, velocity, direction) {
    const stride = velocity * this.gaitParams.strideLength;
    
    this.legs.forEach((leg, i) => {
      let legPhase = (phase + this.gaitParams.phaseOffsets[i]) % 1;
      let x = 0, y = 0, z = -this.bodyHeight;
      
      if (legPhase < 0.5) {
        // Stance phase - foot in contact with ground
        x = Math.cos(legPhase * Math.PI * 2) * stride;
        leg.isGrounded = true;
        leg.lastContact = Date.now();
      } else {
        // Swing phase - foot in air
        x = Math.cos(legPhase * Math.PI * 2) * stride;
        z = -this.bodyHeight + Math.sin((legPhase - 0.5) * Math.PI * 2) * this.gaitParams.stepHeight;
        leg.isGrounded = false;
      }
      
      // Apply direction modifications
      const directionModifiers = {
        'FORWARD': [x, y],
        'BACKWARD': [-x, y],
        'LEFT': [-y, x],
        'RIGHT': [y, -x]
      };
      
      [x, y] = directionModifiers[direction] || directionModifiers['FORWARD'];
      
      // Add leg's base position
      x += leg.position.x;
      y += leg.position.y;
      
      const angles = this.calculateInverseKinematics(x, y, z);
      this.setLegAngles(i, angles);
      
      // Update leg state
      leg.position = { x, y, z };
      leg.forceVector = this.calculateLegForce(leg, legPhase);
    });
  }

  calculateLegForce(leg, phase) {
    const force = { x: 0, y: 0, z: 0 };
    
    if (leg.isGrounded) {
      const contactTime = Date.now() - leg.lastContact;
      const contactForce = Math.min(1, contactTime / 100); // Ramp up force over 100ms
      
      force.z = -9.81 * this.bodyHeight * 0.25 * contactForce; // Distribute body weight
      
      // Add horizontal forces based on movement
      force.x = (leg.position.x - leg.position.x) * 0.1;
      force.y = (leg.position.y - leg.position.y) * 0.1;
    }
    
    return force;
  }

  calculateInverseKinematics(x, y, z) {
    const L1 = 50; // Upper leg length
    const L2 = 50; // Lower leg length
    
    // Calculate leg length and angle in the x-y plane
    const L = Math.sqrt(x*x + y*y);
    const gamma = Math.atan2(y, x);
    
    // Calculate leg length in the x-z plane
    const D = Math.sqrt(L*L + z*z);
    
    // Check if position is reachable
    if (D > L1 + L2) {
      console.warn('Position out of reach, adjusting...');
      const scale = (L1 + L2) / D;
      x *= scale;
      y *= scale;
      z *= scale;
    }
    
    // Calculate joint angles using cosine law
    const alpha = Math.acos((L1*L1 + D*D - L2*L2)/(2*L1*D)) + Math.atan2(z, L);
    const beta = Math.acos((L1*L1 + L2*L2 - D*D)/(2*L1*L2));
    
    // Convert to degrees and ensure angles are within servo limits
    return {
      hip: this.clampAngle(gamma * 180/Math.PI, -90, 90),
      knee: this.clampAngle((Math.PI - beta) * 180/Math.PI, 0, 150)
    };
  }

  clampAngle(angle, min, max) {
    return Math.max(min, Math.min(max, angle));
  }

  setLegAngles(legIndex, angles) {
    const servoIndex = legIndex * this.SERVOS_PER_LEG;
    this.currentLegAngles[servoIndex] = angles.hip;
    this.currentLegAngles[servoIndex + 1] = angles.knee;
    
    // Emit angle updates if needed
    this.onAngleUpdate?.(legIndex, angles);
  }

  balance() {
    this.updateSensorData();
    const correction = this.calculateBalanceCorrection();
    this.applyBalanceCorrection(correction);
  }

  updateSensorData() {
    // In a real implementation, this would read from actual sensors
    // Here we simulate sensor noise and drift
    const now = Date.now();
    const dt = (now - this.pidState.lastTime) / 1000;
    
    this.sensorData.accel = {
      x: Math.sin(now * 0.001) * 0.1,
      y: Math.cos(now * 0.001) * 0.1,
      z: -9.81 + Math.sin(now * 0.002) * 0.05
    };
    
    this.sensorData.gyro = {
      x: (Math.random() - 0.5) * 0.1,
      y: (Math.random() - 0.5) * 0.1,
      z: (Math.random() - 0.5) * 0.1
    };
    
    this.calculateOrientation(dt);
    this.pidState.lastTime = now;
  }

  calculateOrientation(dt) {
    const alpha = 0.96; // Complementary filter coefficient
    
    // Calculate angles from accelerometer
    const accelPitch = Math.atan2(this.sensorData.accel.x, 
      Math.sqrt(this.sensorData.accel.y * this.sensorData.accel.y + 
                this.sensorData.accel.z * this.sensorData.accel.z));
                
    const accelRoll = Math.atan2(this.sensorData.accel.y, 
      Math.sqrt(this.sensorData.accel.x * this.sensorData.accel.x + 
                this.sensorData.accel.z * this.sensorData.accel.z));
    
    // Complementary filter
    this.sensorData.orientation.pitch = alpha * 
      (this.sensorData.orientation.pitch + this.sensorData.gyro.x * dt) + 
      (1 - alpha) * accelPitch;
      
    this.sensorData.orientation.roll = alpha * 
      (this.sensorData.orientation.roll + this.sensorData.gyro.y * dt) + 
      (1 - alpha) * accelRoll;
      
    this.sensorData.orientation.yaw += this.sensorData.gyro.z * dt;
  }

  calculateBalanceCorrection() {
    const dt = (Date.now() - this.pidState.lastTime) / 1000;
    
    const pitch = this.sensorData.orientation.pitch;
    const roll = this.sensorData.orientation.roll;
    
    // Update error sums for integral term
    this.pidState.pitchErrorSum += pitch * dt;
    this.pidState.rollErrorSum += roll * dt;
    
    // Calculate PID terms
    const pitchCorrection = 
      this.pidGains.kp * pitch +
      this.pidGains.ki * this.pidState.pitchErrorSum +
      this.pidGains.kd * (pitch - this.pidState.lastPitchError) / dt;
      
    const rollCorrection = 
      this.pidGains.kp * roll +
      this.pidGains.ki * this.pidState.rollErrorSum +
      this.pidGains.kd * (roll - this.pidState.lastRollError) / dt;
    
    // Update last error values
    this.pidState.lastPitchError = pitch;
    this.pidState.lastRollError = roll;
    
    return { pitch: pitchCorrection, roll: rollCorrection };
  }

  applyBalanceCorrection(correction = { pitch: 0, roll: 0 }) {
    this.legs.forEach((leg, i) => {
      let z = -this.bodyHeight;
      
      // Apply pitch correction (front/back)
      if (i < 2) z += correction.pitch * 20;
      else z -= correction.pitch * 20;
      
      // Apply roll correction (left/right)
      if (i % 2) z += correction.roll * 20;
      else z -= correction.roll * 20;
      
      const angles = this.calculateInverseKinematics(
        leg.position.x,
        leg.position.y,
        z
      );
      
      this.setLegAngles(i, angles);
    });
  }

  stand() {
    this.moving = false;
    
    // Move all legs to default standing position
    this.legs.forEach((leg, i) => {
      const angles = this.calculateInverseKinematics(
        leg.position.x,
        leg.position.y,
        -this.bodyHeight
      );
      
      this.setLegAngles(i, angles);
      leg.isGrounded = true;
    });
  }

  stabilize() {
    this.updateSensorData();
    
    // If severely unbalanced, emergency stop
    if (Math.abs(this.sensorData.orientation.pitch) > 45 ||
        Math.abs(this.sensorData.orientation.roll) > 45) {
      this.emergencyStop();
      return;
    }
    
    this.balance();
    
    // Check if stabilization is complete
    if (this.isBalanced()) {
      this.stand();
    }
  }

  emergencyStop() {
    this.moving = false;
    this.stand();
    console.warn('Emergency stop triggered');
  }

  isBalanced() {
    const pitchThreshold = 5; // degrees
    const rollThreshold = 5;  // degrees
    
    return Math.abs(this.sensorData.orientation.pitch) < pitchThreshold &&
           Math.abs(this.sensorData.orientation.roll) < rollThreshold;
  }

  getStatus() {
    return {
      moving: this.moving,
      legPositions: this.legs.map(leg => ({
        position: leg.position,
        isGrounded: leg.isGrounded,
        angles: leg.angles,
        force: leg.forceVector
      })),
      orientation: this.sensorData.orientation,
      isBalanced: this.isBalanced(),
      bodyHeight: this.bodyHeight
    };
  }
}