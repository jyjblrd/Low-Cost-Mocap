import { max } from "mathjs";
import { MutableRefObject, useEffect, useRef } from "react";
import { Color, InstancedMesh, Object3D } from "three";

export default function Points({objectPoints, objectPointErrors, count}: {objectPoints: MutableRefObject<number[][]>, objectPointErrors: MutableRefObject<number[]>, count: number}) {
  const instancedMeshRef = useRef<InstancedMesh>()
  const temp = new Object3D()
  const tempColour = new Color()
  const maxError = objectPointErrors.current.length !== 0 ? max(objectPointErrors.current) : 1

  const errorToColour = (error: number) => {
    const scaledError = error/maxError
    const logError = scaledError/(0.1+scaledError)

    return tempColour.set(0x009999 + Math.round(logError*0xff)*0x10000)
  }

  useEffect(() => {
    objectPoints.current.forEach(([x, y, z]: Array<number>, i) => {
      temp.position.set(x, y, z)
      temp.updateMatrix()
      instancedMeshRef.current!.setMatrixAt(i, temp.matrix)
      instancedMeshRef.current!.setColorAt(i, errorToColour(objectPointErrors.current[i]))
    })
    instancedMeshRef.current!.instanceMatrix.needsUpdate = true
  }, [count])
  return (
    <instancedMesh ref={instancedMeshRef} args={[undefined, undefined, objectPoints.current.length]}>
      <sphereGeometry args={[0.01, 4, 4]}/>
      <meshLambertMaterial />
    </instancedMesh>
  )
}