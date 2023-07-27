"use client";

import { FormEventHandler, useState, createRef, Ref } from 'react';
import { Button, Card, Col, Container, Row } from 'react-bootstrap';
import Toolbar from './components/Toolbar';
import Form from 'react-bootstrap/Form';
import { Tooltip } from 'react-tooltip'
import SceneViewer from './components/SceneViewer';
import { io } from 'socket.io-client';

// const socket = io("http://localhost:3000", {path: "/api/socket.io"});
// console.log(socket)

export default function App() {
  const [cameraStreamRunning, setCameraStreamRunning] = useState(false);

  const [exposure, setExposure] = useState(100);
  const [gain, setGain] = useState(0);

  const [capturingPointsForPose, setCapturingPointsForPose] = useState(false);
  const [capturedPointsForPose, setCapturedPointsForPose] = useState("");
  const [liveMocap, setLiveMocap] = useState(false);

  const [rotationMatrix, setRotationMatrix] = useState<Array<Array<number>>>()
  const [translationMatrix, setTranslationMatrix] = useState<Array<number>>()

  const updateCameraSettings: FormEventHandler = (e) => {
    e.preventDefault()
    
    fetch('/api/update-camera-settings', {
      method: 'POST',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        exposure,
        gain,
      })
    })
  }

  const capturePointsForPose = async (startOrStop: string) => {
    console.log("daw")
    //socket.io.emit("capture-points", { startOrStop })
  }

  // socket.io.on("captured-points", (data) => {
  // })

  const calculateCameraPose = async (cameraPoints: Array<Array<Array<number>>>) => {
    const response = await fetch('/api/calculate-camera-pose', {
      method: 'POST',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        cameraPoints
      })
    })
    const {R, t} = await response.json()
    setRotationMatrix(R)
    setTranslationMatrix(t)
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
                <img src={cameraStreamRunning ? "/api/camera-stream" : ""} />
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
                  value={capturingPointsForPose ? "Capturing Points..." : capturedPointsForPose}
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
                  disabled={!(isValidJson(capturedPointsForPose) && JSON.parse(capturedPointsForPose).length !== 0)}
                  onClick={() => {
                    calculateCameraPose(JSON.parse(capturedPointsForPose))
                  }}>
                  Calculate Camera Pose with {isValidJson(capturedPointsForPose) ? JSON.parse(capturedPointsForPose).length : 0} points
                </Button>
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col>
                <h6>Rotation Matrix</h6>
                {rotationMatrix}
                <h6>Translation Matrix</h6>
                {translationMatrix}
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
                  variant={liveMocap ? "outline-danger" : "outline-primary"}
                  disabled={!cameraStreamRunning}
                  onClick={() => {
                    setCapturingPointsForPose(!liveMocap);
                    startLiveMocap(liveMocap ? "stop" : "start");
                  }
                }>
                  {liveMocap ? "Stop" : "Start"}
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
                <h4>Scene Viewer</h4>
              </Col>
            </Row>
            <Row>
              <Col>
                <SceneViewer cameras={rotationMatrix ? [{R: [[1,0,0],[0,1,0],[0,0,1]], t: [0,0,0]}, {R: rotationMatrix, t: translationMatrix!}] : []}/>
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
    </Container>
  )
}
