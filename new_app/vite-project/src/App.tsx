"use client";

import { FormEventHandler, useState, createRef, Ref, useRef, useEffect } from 'react';
import { Button, Card, Col, Container, Row } from 'react-bootstrap';
import Toolbar from './components/Toolbar';
import Form from 'react-bootstrap/Form';
import { Tooltip } from 'react-tooltip'
import CameraWireframe from './components/CameraWireframe';
import { io } from 'socket.io-client';
import { Canvas, useFrame } from '@react-three/fiber'
import { Stats, OrbitControls } from '@react-three/drei'
import Points from './components/Points';
import { socket } from './shared/styles/scripts/socket';
import { matrix, mean, multiply, rotationMatrix } from 'mathjs';
import Objects from './components/Objects';
import Chart from './components/chart';

export default function App() {
  const [cameraStreamRunning, setCameraStreamRunning] = useState(false);

  const [exposure, setExposure] = useState(100);
  const [gain, setGain] = useState(0);

  const [capturingPointsForPose, setCapturingPointsForPose] = useState(false);
  const [capturedPointsForPose, setCapturedPointsForPose] = useState("");
  
  const [isTriangulatingPoints, setIsTriangulatingPoints] = useState(false);
  const [isLocatingObjects, setIsLocatingObjects] = useState(false);

  const objectPoints = useRef<Array<Array<Array<number>>>>([])
  const filteredPoints = useRef<number[][]>([])
  const droneSetpointHistory = useRef<number[][]>([])
  const objectPointErrors = useRef<Array<Array<number>>>([])
  const objects = useRef<Array<Array<Object>>>([])
  const [objectPointCount, setObjectPointCount] = useState(0);

  const [cameraPoses, setCameraPoses] = useState<Array<object>>([])
  const [toWorldCoordsMatrix, setToWorldCoordsMatrix] = useState<number[][]>([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

  const [droneArmed, setDroneArmed] = useState(false)
  const [dronePID, setDronePID] = useState(["0.2","0.03","0.05","0.2","0.03","0.05","0.6","0.3","0.1","0.3","0.1","0.05","1","1"])
  const [droneSetpoint, setDroneSetpoint] = useState(["0","0","0"])
  const [droneSetpointWithMotion, setDroneSetpointWithMotion] = useState([0,0,0])
  const [droneTrim, setDroneTrim] = useState(["0","0","0","0"])

  const [motionPreset, setMotionPreset] = useState("setpoint")

  const updateCameraSettings: FormEventHandler = (e) => {
    e.preventDefault()
    socket.emit("update-camera-settings", {
      exposure,
      gain,
    })
  }

  const capturePointsForPose = async (startOrStop: string) => {
    if (startOrStop === "start") {
      setCapturedPointsForPose("")
    }
    socket.emit("capture-points", { startOrStop })
  }

  useEffect(() => {
    socket.on("image-points", (data) => {
      setCapturedPointsForPose(`${capturedPointsForPose}${JSON.stringify(data)},`)
    })

    return () => {
      socket.off("image-points")
    }
  }, [capturedPointsForPose])

  useEffect(() => {
    socket.emit("arm-drone", { droneArmed })
    let count = 0
    const pingInterval = setInterval(() => {
      count += 1
      socket.emit("arm-drone", { droneArmed, count })
    }, 500)

    return () => {
      clearInterval(pingInterval)
    }
  }, [droneArmed])

  useEffect(() => {
    socket.emit("set-drone-pid", { dronePID })
  }, [dronePID])

  useEffect(() => {
    socket.emit("set-drone-trim", { droneTrim })
  }, [droneTrim])

  useEffect(() => {
    let timestamp = Date.now()/1000

    if (motionPreset !== "setpoint") {
      const motionInterval = setInterval(() => {
        timestamp = Date.now()/1000
        let tempDroneSetpoint = [] as number[]

        switch (motionPreset) {
          case "circle": {
            const radius = 0.3
            const period = 10
            tempDroneSetpoint = [
              parseFloat(droneSetpoint[0]) + radius*Math.cos(timestamp*2*Math.PI / period), 
              parseFloat(droneSetpoint[1]) + radius*Math.sin(timestamp*2*Math.PI / period), 
              parseFloat(droneSetpoint[2])
            ]
            socket.emit("set-drone-setpoint", { "droneSetpoint": tempDroneSetpoint })
            break;
          }
        
          case "square": {
            const size = 0.2
            const period = 20
            let offset = [0,0]
            switch (Math.floor((timestamp*4)/period) % 4) {
              case 0:
                offset = [1,1]
                break
              case 1:
                offset = [1,-1]
                break
              case 2:
                offset = [-1,-1]
                break
              case 3:
                offset = [-1,1]
                break
            }

            tempDroneSetpoint = [
              parseFloat(droneSetpoint[0]) + (offset[0] * size), 
              parseFloat(droneSetpoint[1]) + (offset[1] * size), 
              parseFloat(droneSetpoint[2])
            ]
            socket.emit("set-drone-setpoint", { "droneSetpoint": tempDroneSetpoint })
            break;
          }

          default:
            break;
        }

        setDroneSetpointWithMotion(tempDroneSetpoint)
      }, 100)

      return () => {
        clearInterval(motionInterval)
      }
    }
    else {
      setDroneSetpointWithMotion(droneSetpoint.map(x => parseFloat(x)))
      socket.emit("set-drone-setpoint", { droneSetpoint })
    }
  }, [motionPreset, droneSetpoint])

  useEffect(() => {
    socket.on("to-world-coords-matrix", (data) => {
      setToWorldCoordsMatrix(data["to_world_coords_matrix"])
      setObjectPointCount(objectPointCount+1) 
    })

    return () => {
      socket.off("to-world-coords-matrix")
    }
  }, [objectPointCount])

  useEffect(() => {
    socket.on("object-points", (data) => {
      objectPoints.current.push(data["object_points"])
      if (data["filtered_object"][0].length != 0) {
        filteredPoints.current.push(data["filtered_object"][0])
      }
      objectPointErrors.current.push(data["errors"])
      objects.current.push(data["objects"])
      droneSetpointHistory.current.push(droneSetpointWithMotion)
      setObjectPointCount(objectPointCount+1)
    })

    return () => {
      socket.off("object-points")
    }
  }, [objectPointCount])

  useEffect(() => {
    socket.on("camera-pose", data => {
      console.log(data["camera_poses"])
      setCameraPoses(data["camera_poses"])
    })

    return () => {
      socket.off("camera-pose")
    }
  }, [])

  const toHomogeneous = (point3: number[]) => {
    return [...point3, 1]
  }

  const fromHomogeneous = (point4: number[]) => {
    return point4.slice(0,3).map(x => x/point4[3])
  }

  const calculateCameraPose = async (cameraPoints: Array<Array<Array<number>>>) => {
    socket.emit("calculate-camera-pose", { cameraPoints })
  }

  const isValidJson = (str: string) => {
    try {
      const o = JSON.parse(str);
      if (o && typeof o === "object") {
        return true;
      }
    } catch (e) {}
    return false;
  }

  const startLiveMocap = (startOrStop: string) => {
    socket.emit("triangulate-points", { startOrStop, cameraPoses, toWorldCoordsMatrix })
  }

  return (
    <Container fluid>
      <Row className="mt-3 mb-3 flex-nowrap" style={{ alignItems: 'center' }}>
        <Col className="ms-4" style={{ width: 'fit-content' }} md="auto">
          <h2>MoCap</h2>
        </Col>
        <Col>
          <Toolbar/>
        </Col>
      </Row>
      <Row>
        <Col>
          <Card className='shadow-sm p-3'>
            <Row>
              <Col xs="auto">
                <h4>Camera Stream</h4>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  variant={cameraStreamRunning ? "outline-danger" : "outline-primary"}
                  onClick={() => {
                    setCameraStreamRunning(!cameraStreamRunning);
                  }
                }>
                  {cameraStreamRunning ? "Stop" : "Start"}
                </Button>
              </Col>
            </Row>
            <Row className='mt-2 mb-1' style={{height: "320px"}}>
              <Col>
                <img src={cameraStreamRunning ? "http://localhost:3001/api/camera-stream" : ""} />
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
      <Row className='pt-3'>
        <Col xs={4}>
          <Card className='shadow-sm p-3 h-100'>
            <Row>
              <Col xs="auto">
                <h4>Camera Settings</h4>
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs="4">
                <Form onChange={updateCameraSettings} className='ps-3'>
                  <Form.Group className="mb-1">
                    <Form.Label>Exposure: {exposure}</Form.Label>
                    <Form.Range value={exposure} onChange={(event) => setExposure(parseFloat(event.target.value))}/>
                  </Form.Group>
                  <Form.Group className="mb-1">
                    <Form.Label>Gain: {gain}</Form.Label>
                    <Form.Range value={gain} onChange={(event) => setGain(parseFloat(event.target.value))}/>
                  </Form.Group>
                </Form>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Live Triangulation</h4>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  variant={isTriangulatingPoints ? "outline-danger" : "outline-primary"}
                  disabled={!cameraStreamRunning}
                  onClick={() => {
                    if (!isTriangulatingPoints) {
                      objectPoints.current = []
                      objectPointErrors.current = []
                      objects.current = []
                      filteredPoints.current = []
                      droneSetpointHistory.current = []
                    }
                    setIsTriangulatingPoints(!isTriangulatingPoints);
                    startLiveMocap(isTriangulatingPoints ? "stop" : "start");
                  }
                }>
                  {isTriangulatingPoints ? "Stop" : "Start"}
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Locate Objects</h4>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  variant={isLocatingObjects ? "outline-danger" : "outline-primary"}
                  disabled={!cameraStreamRunning}
                  onClick={() => {
                    setIsLocatingObjects(!isLocatingObjects);
                    socket.emit("locate-objects", { startOrStop: isLocatingObjects ? "stop" : "start" })
                  }
                }>
                  {isLocatingObjects ? "Stop" : "Start"}
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Set Scale Using Points</h4>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  variant="outline-primary"
                  disabled={!isTriangulatingPoints && objectPoints.current.length == 0}
                  onClick={() => {
                    socket.emit("determine-scale", { objectPoints: objectPoints.current, cameraPoses: cameraPoses })
                  }
                }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Acquire Floor</h4>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  variant="outline-primary"
                  disabled={!isTriangulatingPoints && objectPoints.current.length == 0}
                  onClick={() => {
                    socket.emit("acquire-floor", { objectPoints: objectPoints.current })
                  }
                }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Set Origin</h4>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  variant="outline-primary"
                  disabled={!isTriangulatingPoints && objectPoints.current.length == 0}
                  onClick={() => {
                    socket.emit("set-origin", { objectPoint: objectPoints.current[0][0], toWorldCoordsMatrix })
                  }
                }>
                  Run
                </Button>
              </Col>
            </Row>
          </Card>
        </Col>
        <Col xs={4}>
          <Card className='shadow-sm p-3 h-100'>
            <Row>
              <Col xs="auto">
                <h4>Collect points for camera pose calibration</h4>
              </Col>
              <Col>
                <Tooltip id="collect-points-for-pose-button-tooltip" />
                <a data-tooltip-hidden={cameraStreamRunning} data-tooltip-variant='error' data-tooltip-id='collect-points-for-pose-button-tooltip' data-tooltip-content="Start camera stream first">
                  <Button
                    size='sm' 
                    variant={capturingPointsForPose ? "outline-danger" : "outline-primary"}
                    disabled={!cameraStreamRunning}
                    onClick={() => {
                      setCapturingPointsForPose(!capturingPointsForPose);
                      capturePointsForPose(capturingPointsForPose ? "stop" : "start");
                    }
                  }>
                    {capturingPointsForPose ? "Stop" : "Start"}
                  </Button>
                </a>
              </Col>
            </Row>
            <Row>
              <Col>
                <Form.Control 
                  as="textarea" 
                  rows={5} 
                  value={capturedPointsForPose}
                  onChange={(event) => setCapturedPointsForPose(event.target.value)}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col>
                <Button
                  size='sm' 
                  className='float-end'
                  variant="outline-primary"
                  disabled={!(isValidJson(`[${capturedPointsForPose.slice(0,-1)}]`) && JSON.parse(`[${capturedPointsForPose.slice(0,-1)}]`).length !== 0)}
                  onClick={() => {
                    calculateCameraPose(JSON.parse(`[${capturedPointsForPose.slice(0,-1)}]`))
                  }}>
                  Calculate Camera Pose with {isValidJson(`[${capturedPointsForPose.slice(0,-1)}]`) ? JSON.parse(`[${capturedPointsForPose.slice(0,-1)}]`).length : 0} points
                </Button>
              </Col> 
            </Row>
            <Row>
              <Col>
                <Form.Control 
                  value={JSON.stringify(cameraPoses)}
                  onChange={(event) => setCameraPoses(JSON.parse(event.target.value))}
                />
              </Col>
            </Row>
            <Row>
              <Col>
                <Form.Control 
                  value={JSON.stringify(toWorldCoordsMatrix)}
                  onChange={(event) => setToWorldCoordsMatrix(JSON.parse(event.target.value))}
                />
              </Col>
            </Row>
          </Card>
        </Col>
        <Col xs={4}>
          <Card className='shadow-sm p-3 h-100'>
            <Row>
              <Col xs="auto">
                <h4>Arm Drone</h4>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  variant={droneArmed ? "outline-danger" : "outline-primary"}
                  onClick={() => {
                    setDroneArmed(!droneArmed);
                  }
                }>
                  {droneArmed ? "Disarm" : "Arm"}
                </Button>
              </Col>
            </Row>
            <Row className='pt-2'>
              <Col xs={2} className='pt-2'>
                Setpoint
              </Col>
              <Col>
                <Form.Control 
                  value={droneSetpoint[0]}
                  onChange={(event) => {
                    let newDroneSetpoint = droneSetpoint.slice()
                    newDroneSetpoint[0] = event.target.value
                    setDroneSetpoint(newDroneSetpoint)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={droneSetpoint[1]}
                  onChange={(event) => {
                    let newDroneSetpoint = droneSetpoint.slice()
                    newDroneSetpoint[1] = event.target.value
                    setDroneSetpoint(newDroneSetpoint)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={droneSetpoint[2]}
                  onChange={(event) => {
                    let newDroneSetpoint = droneSetpoint.slice()
                    newDroneSetpoint[2] = event.target.value
                    setDroneSetpoint(newDroneSetpoint)
                  }}
                />
              </Col>
            </Row><Row className='pt-2'>
              <Col>
                <Button
                  size='sm' 
                  onClick={() => {
                    setMotionPreset("setpoint");
                  }
                }>
                  Setpoint
                </Button>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  onClick={() => {
                    setMotionPreset("circle");
                  }
                }>
                  Circle
                </Button>
              </Col>
              <Col>
                <Button
                  size='sm' 
                  onClick={() => {
                    setMotionPreset("square");
                  }
                }>
                  Square
                </Button>
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={{offset:1}} className='text-center'>
                XY Pos P
              </Col>
              <Col className='text-center'>
                Z Pos P
              </Col>
            </Row>
            <Row className='pt-2'>
              <Col>
                <Form.Control 
                  value={dronePID[12]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[12] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[13]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[13] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={{offset:1}} className='text-center'>
                P
              </Col>
              <Col className='text-center'>
                I
              </Col>
              <Col className='text-center'>
                D
              </Col>
            </Row>
            <Row className='pt-2'>
              <Col xs={1} className='pt-2 text-end'>
                X
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[0]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[0] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[1]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[1] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[2]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[2] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={1} className='pt-2 text-end'>
                Y
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[3]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[3] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[4]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[4] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[5]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[5] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={1} className='pt-2 text-end'>
                Z
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[6]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[6] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[7]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[7] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[8]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[8] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={1} className='pt-2 text-end'>
                YAW
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[9]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[9] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[10]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[10] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control 
                  value={dronePID[11]}
                  onChange={(event) => {
                      let newDronePID = dronePID.slice()
                      newDronePID[11] = event.target.value
                      setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row>
              <Col>
                <Form.Group className="mb-1">
                  <Form.Label>X Trim: {droneTrim[0]}</Form.Label>
                  <Form.Range value={droneTrim[0]} min={-800} max={800} onChange={(event) => {
                    let newDroneTrim = droneTrim.slice()
                    newDroneTrim[0] = event.target.value
                    setDroneTrim(newDroneTrim)
                  }}/>
                </Form.Group>
                <Form.Group className="mb-1">
                  <Form.Label>Y Trim: {droneTrim[1]}</Form.Label>
                  <Form.Range value={droneTrim[1]} min={-800} max={800} onChange={(event) => {
                    let newDroneTrim = droneTrim.slice()
                    newDroneTrim[1] = event.target.value
                    setDroneTrim(newDroneTrim)
                  }}/>
                </Form.Group>
                <Form.Group className="mb-1">
                  <Form.Label>Z Trim: {droneTrim[2]}</Form.Label>
                  <Form.Range value={droneTrim[2]} min={-800} max={800} onChange={(event) => {
                    let newDroneTrim = droneTrim.slice()
                    newDroneTrim[2] = event.target.value
                    setDroneTrim(newDroneTrim)
                  }}/>
                </Form.Group>
                <Form.Group className="mb-1">
                  <Form.Label>Yaw Trim: {droneTrim[3]}</Form.Label>
                  <Form.Range value={droneTrim[3]} min={-800} max={800} onChange={(event) => {
                    let newDroneTrim = droneTrim.slice()
                    newDroneTrim[3] = event.target.value
                    setDroneTrim(newDroneTrim)
                  }}/>
                </Form.Group>
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
      <Row className='pt-3'>
        <Col>
          <Card className='shadow-sm p-3'>
            <Chart filteredPointsRef={filteredPoints} droneSetpointHistoryRef={droneSetpointHistory} xyPosKp={parseFloat(dronePID[12])} zPosKp={parseFloat(dronePID[13])} />
          </Card>
        </Col>
      </Row>
      <Row className='pt-3'>
        <Col>
          <Card className='shadow-sm p-3'>
            <Row>
              <Col xs="auto">
                {/* <h4>Scene Viewer {objectPointErrors.current.length !== 0 ? mean(objectPointErrors.current.flat()) : ""}</h4> */}
              </Col>
            </Row>
            <Row>
              <Col style={{height: "1000px"}}>
                <Canvas>
                  <ambientLight/>
                  {cameraPoses.map(({R, t}, i) => (
                      <CameraWireframe R={R} t={t} toWorldCoordsMatrix={toWorldCoordsMatrix} key={i}/>
                  ))}
                  <Points objectPointsRef={objectPoints} objectPointErrorsRef={objectPointErrors} count={objectPointCount} toWorldCoordsMatrix={toWorldCoordsMatrix}/>
                  {/* <Objects objectsRef={objects} count={objectPointCount}/> */}
                  <OrbitControls />
                  <axesHelper args={[0.2]}/>
                  <gridHelper/>
                </Canvas>
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
    </Container>
  )
}
