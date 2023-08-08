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
import { matrix, mean, multiply } from 'mathjs';

export default function App() {
  const [cameraStreamRunning, setCameraStreamRunning] = useState(false);

  const [exposure, setExposure] = useState(100);
  const [gain, setGain] = useState(0);

  const [capturingPointsForPose, setCapturingPointsForPose] = useState(false);
  const [capturedPointsForPose, setCapturedPointsForPose] = useState("");
  
  const [isTriangulatingPoints, setIsTriangulatingPoints] = useState(false);

  const objectPoints = useRef<Array<Array<number>>>([])
  const objectPointErrors = useRef<Array<number>>([])
  const [objectPointCount, setObjectPointCount] = useState(0);

  const [cameraPoses, setCameraPoses] = useState<Array<object>>([])

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
    socket.on("object-point", (data) => {
      const rotated_point = data["object_points"].map(objectPoint => multiply(matrix([[-1,0,0],[0,-1,0],[0,0,1]]), objectPoint)._data)
      objectPoints.current = objectPoints.current.concat(rotated_point)
      objectPointErrors.current = objectPointErrors.current.concat(data["errors"])
      setObjectPointCount(objectPointCount+1)
    })

    return () => {
      socket.off("object-point")
    }
  }, [objectPointCount])

  useEffect(() => {
    socket.on("camera-pose", data => {
      console.log(data["error"])
      setCameraPoses(data["camera_poses"])
    })

    return () => {
      socket.off("camera-pose")
    }
  }, [])

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
    socket.emit("triangulate-points", { startOrStop, cameraPoses })
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
        <Col xs="4">
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
          </Card>
        </Col>
        <Col xs="4">
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
          </Card>
        </Col>
        <Col xs="4">
          <Card className='shadow-sm p-3 h-100'>
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
                    }
                    setIsTriangulatingPoints(!isTriangulatingPoints);
                    startLiveMocap(isTriangulatingPoints ? "stop" : "start");
                  }
                }>
                  {isTriangulatingPoints ? "Stop" : "Start"}
                </Button>
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
      <Row className='pt-3'>
        <Col>
          <Card className='shadow-sm p-3'>
            <Row>
              <Col xs="auto">
                <h4>Scene Viewer {objectPointErrors.current.length !== 0 ? mean(objectPointErrors.current) : ""}</h4>
              </Col>
            </Row>
            <Row>
              <Col style={{height: "1000px"}}>
                <Canvas>
                  <ambientLight/>
                  {cameraPoses.map(({R, t}, i) => (
                    <CameraWireframe R={R} t={t} key={i}/>
                  ))}
                  <Points objectPoints={objectPoints} objectPointErrors={objectPointErrors} count={objectPointCount}/>
                  <OrbitControls />
                </Canvas>
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
    </Container>
  )
}
