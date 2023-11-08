import { max, multiply } from "mathjs";
import { MutableRefObject, useEffect, useRef } from "react";
import { Color, InstancedMesh, Matrix4, Object3D } from "three";

export default function Points({objectPointsRef, objectPointErrorsRef, count}: {objectPointsRef: MutableRefObject<number[][][]>, objectPointErrorsRef: MutableRefObject<number[][]>, count: number}) {
  const objectPoints = objectPointsRef.current.flat()
  const objectPointErrors = objectPointErrorsRef.current.flat()

  const instancedMeshRef = useRef<InstancedMesh>()
  const temp = new Object3D()
  const tempColour = new Color()
  const maxError = objectPointErrors.length !== 0 ? max(objectPointErrors) : 1

  const errorToColour = (error: number) => {
    const scaledError = error/maxError
    const logError = scaledError/(0.1+scaledError)

    return tempColour.set(0x009999 + Math.round(logError*0xff)*0x10000)
  }

  useEffect(() => {
    objectPoints.forEach(([x, y, z]: Array<number>, i) => {
      temp.position.set(x, z, y) // y is up in threejs
      temp.updateMatrix()
      instancedMeshRef.current!.setMatrixAt(i, temp.matrix)
      instancedMeshRef.current!.setColorAt(i, errorToColour(objectPointErrors[i]))
    })
    instancedMeshRef.current!.instanceMatrix.needsUpdate = true
  }, [count])
  return (
    <instancedMesh ref={instancedMeshRef} args={[undefined, undefined, objectPoints.length]}>
      <sphereGeometry args={[0.008, 4, 4]}/>
      <meshLambertMaterial />
    </instancedMesh>
  )
}