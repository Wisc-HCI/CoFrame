const _AXES2TUPLE = {
    sxyz: [0, 0, 0, 0], sxyx: [0, 0, 1, 0], sxzy: [0, 1, 0, 0],
    sxzx: [0, 1, 1, 0], syzx: [1, 0, 0, 0], syzy: [1, 0, 1, 0],
    syxz: [1, 1, 0, 0], syxy: [1, 1, 1, 0], szxy: [2, 0, 0, 0],
    szxz: [2, 0, 1, 0], szyx: [2, 1, 0, 0], szyz: [2, 1, 1, 0],
    rzyx: [0, 0, 0, 1], rxyx: [0, 0, 1, 1], ryzx: [0, 1, 0, 1],
    rxzx: [0, 1, 1, 1], rxzy: [1, 0, 0, 1], ryzy: [1, 0, 1, 1],
    rzxy: [1, 1, 0, 1], ryxy: [1, 1, 1, 1], ryxz: [2, 0, 0, 1],
    rzxz: [2, 0, 1, 1], rxyz: [2, 1, 0, 1], rzyz: [2, 1, 1, 1]}

const _NEXT_AXIS = [1, 2, 0, 1]

const _EPS = 8.881784197001252*Math.pow(10,-16)

const dot = (a, b) => a.map((x, i) => a[i] * b[i]).reduce((m, n) => m + n);

const outer = (a,b) => a.map((x) => b.map(y => x*y));

const quaternionFromEuler = (vec3, axes='szxy') => {
  /*
  Return quaternion from Euler angles and axis sequence.
  ai, aj, ak : Euler's roll, pitch and yaw angles
  axes : One of 24 axis sequences as string or encoded tuple
  >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
  >>> numpy.allclose(q, [0.435953, 0.310622, -0.718287, 0.444435])
  True
  */
  let [ai, aj, ak] = [...vec3];
  let [firstaxis, parity, repetition, frame] = [..._AXES2TUPLE[axes.toLowerCase()]];

  let i = firstaxis + 1;
  let j = _NEXT_AXIS[i+parity-1] + 1;
  let k = _NEXT_AXIS[i-parity] + 1;

  if (frame !== 0) {
    let values = [...[ai,ak]]
    ai = values[1]
    ak = values[0]
  }
  if (parity !== 0) {
    aj = -1*aj
  }

  ai = ai/2.0;
  aj = aj/2.0;
  ak = ak/2.0;
  let ci = Math.cos(ai);
  let si = Math.sin(ai);
  let cj = Math.cos(aj);
  let sj = Math.sin(aj);
  let ck = Math.cos(ak);
  let sk = Math.sin(ak);
  let cc = ci*ck;
  let cs = ci*sk;
  let sc = si*ck;
  let ss = si*sk;

  let q = [null,null,null,null];
  if (repetition !== 0) {
    q[0] = cj*(cc - ss)
    q[i] = cj*(cs + sc)
    q[j] = sj*(cc + ss)
    q[k] = sj*(cs - sc)
  } else {
    q[0] = cj*cc + sj*ss
    q[i] = cj*sc - sj*cs
    q[j] = cj*ss + sj*cc
    q[k] = cj*cs - sj*sc
  }

  if (parity !== 0) {
    q[j] *= -1.0
  }

  return q
}

const eulerFromMatrix = (matrix, axes='szxy') => {
    /*
    Return Euler angles from rotation matrix for specified axis sequence.
    axes : One of 24 axis sequences as string or encoded tuple
    Note that many Euler angle triplets can describe one matrix.
    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print(axes, "failed")
    */
    let [firstaxis, parity, repetition, frame] = [..._AXES2TUPLE[axes.toLowerCase()]];

    let i = firstaxis;
    let j = _NEXT_AXIS[i+parity];
    let k = _NEXT_AXIS[i-parity+1];

    let ax = 0.0;
    let ay = 0.0;
    let az = 0.0;

    let M = JSON.parse(JSON.stringify(matrix));

    if (repetition !== 0) {
      let sy = Math.sqrt(M[i][j]*M[i][j] + M[i][k]*M[i][k]);
      if (sy > _EPS) {
        ax = Math.atan2( M[i][j],  M[i][k])
        ay = Math.atan2( sy,       M[i][i])
        az = Math.atan2( M[j][i], -M[k][i])
      } else {
        ax = Math.atan2(-M[j][k],  M[j][j]);
        ay = Math.atan2( sy,       M[i][i]);
        az = 0.0;
      }
    } else {
      let cy = Math.sqrt(M[i][i]*M[i][i] + M[j][i]*M[j][i]);
      if (cy > _EPS) {
        ax = Math.atan2( M[k][j],  M[k][k]);
        ay = Math.atan2(-M[k][i],  cy);
        az = Math.atan2( M[j][i],  M[i][i]);
      } else {
        ax = Math.atan2(-M[j][k],  M[j][j]);
        ay = Math.atan2(-M[k][i],  cy);
        az = 0.0;
      }
    }

    if (parity !== 0) {
      [ax, ay, az] = [...[-ax, -ay, -az]]
    }
    if (frame !== 0) {
      [ax, az] = [...[az, ax]]
    }
    return [ax, ay, az]
}

const quaternionMatrix = (quaternion) => {
    /*
    Return homogeneous rotation matrix from quaternion.
    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True
    */
    let q = [...quaternion];
    let n = dot(q, q);
    if (n < _EPS) {
      return [[1,0,0,0],
              [0,1,0,0],
              [0,0,1,0],
              [0,0,0,1]]
    }

    q = q.map(v=>v*Math.sqrt(2.0 / n));
    q = outer(q, q);
    return [[1.0-q[2][2]-q[3][3],     q[1][2]-q[3][0],     q[1][3]+q[2][0], 0.0],
            [    q[1][2]+q[3][0], 1.0-q[1][1]-q[3][3],     q[2][3]-q[1][0], 0.0],
            [    q[1][3]-q[2][0],     q[2][3]+q[1][0], 1.0-q[1][1]-q[2][2], 0.0],
            [                0.0,                 0.0,                 0.0, 1.0]];
}

const eulerFromQuaternion = (quaternion, axes='szxy') => {
  return eulerFromMatrix(quaternionMatrix(quaternion), axes)
}

const quaternionLog = (quaternion) => {
  /*
  Returns the log map vec3 of quaternion
  quaternion list is [w x y z]
  :param quaternion:
  :return:
  */
  let v = [quaternion[1], quaternion[2], quaternion[3]];
  if (Math.abs(quaternion[0]) < 1.0) {
    let a = Math.acos(quaternion[0]);
    if (isNaN(a)) {
      a = 1.0;
    }
    let sina = Math.sin(a)
    if (Math.abs(sina) >= 0.005) {
      let c = a/sina
      v[0] *= c
      v[1] *= c
      v[2] *= c
    }
  }
  return v
}

const quaternionExp = (vec3) => {
    /*
    Returns the exponentiated quaternion from rotation vector vec3
    :param vec3:
    :return: quaternion in format [w x y z]
    */
    let q = [1.0, vec3[0], vec3[1], vec3[2]];
    let a = Math.sqrt(Math.pow(q[0],2)+Math.pow(q[1],2)+Math.pow(q[2],2)+Math.pow(q[3],2));
    let sina = Math.sin(a);
    if (Math.abs(sina) >= 0.005) {
      let c = sina/a;
      q[1] *= c;
      q[2] *= c;
      q[3] *= c;
    }

    q[0] = Math.cos(a)

    return q
}

const quaternionSlerp = (quat0, quat1, fraction, spin=0, shortestpath=true) => {
  /*Return spherical linear interpolation between two quaternions.
  >>> q0 = random_quaternion()
  >>> q1 = random_quaternion()
  >>> q = quaternion_slerp(q0, q1, 0)
  >>> numpy.allclose(q, q0)
  True
  >>> q = quaternion_slerp(q0, q1, 1, 1)
  >>> numpy.allclose(q, q1)
  True
  >>> q = quaternion_slerp(q0, q1, 0.5)
  >>> angle = math.acos(numpy.dot(q0, q))
  >>> numpy.allclose(2, math.acos(numpy.dot(q0, q1)) / angle) or \
      numpy.allclose(2, math.acos(-numpy.dot(q0, q1)) / angle)
  True
  */
  let q0 = unitVector(quat0);
  let q1 = unitVector(quat1);
  if (fraction === 0.0) {
    return q0
  } else if (fraction === 1.0) {
    return q1
  }

  let d = dot(q0, q1);

  if (Math.abs(Math.abs(d) - 1.0) < _EPS) {
    return q0
  }
  if (shortestpath && d < 0.0) {
    // invert rotation
    d = -d;
    q1 = q1.map(v=>-1*v);
  }

  let angle = Math.acos(d) + spin * Math.PI;
  if (Math.abs(angle) < _EPS) {
    return q0
  }

  let isin = 1.0 / Math.sin(angle);
  q0 = q0.map(v=>v*Math.sin((1.0 - fraction) * angle) * isin);
  q1 = q1.map(v=>v*Math.sin(fraction * angle) * isin);
  q0 = q0.map((v,i)=>v*q1[i]);
  return q0
}

const unitVector = (data) => {
  /*Return normalized by length, i.e. Euclidean norm, along axis.
  */
  let scale = Math.sqrt(dot(data,data))
  return data.map(v=>v*scale);
}


export {quaternionFromEuler, eulerFromQuaternion, quaternionExp, quaternionLog, quaternionSlerp};
