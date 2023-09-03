import { max, multiply } from "mathjs";
import { MutableRefObject, useEffect, useRef } from "react";
import { Color, InstancedMesh, Matrix4, Object3D } from "three";

export default function TrajectoryPlanningSetpoints({trajectoryPlanningSetpoints}: {trajectoryPlanningSetpoints: number[][]}) {
  const instancedMeshRef = useRef<InstancedMesh>()
  const temp = new Object3D()
  const tempColour = new Color()

  useEffect(() => {
    trajectoryPlanningSetpoints.forEach(([x, y, z]: Array<number>, i) => {
        temp.position.set(x, z, y) // y is up in threejs
        temp.updateMatrix()
        instancedMeshRef.current!.setMatrixAt(i, temp.matrix)
        instancedMeshRef.current!.setColorAt(i, tempColour.set(0x00ffff))
    })
    instancedMeshRef.current!.instanceMatrix.needsUpdate = true
  }, [trajectoryPlanningSetpoints])

  return (
    <instancedMesh ref={instancedMeshRef} args={[undefined, undefined, trajectoryPlanningSetpoints.length]}>
      <sphereGeometry args={[0.005, 4, 4]}/>
      <meshLambertMaterial />
    </instancedMesh>
  )
}