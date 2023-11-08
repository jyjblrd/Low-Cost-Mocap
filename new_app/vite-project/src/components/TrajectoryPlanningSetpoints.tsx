import { max, multiply } from "mathjs";
import { MutableRefObject, useEffect, useRef } from "react";
import { Color, InstancedMesh, Matrix4, Object3D } from "three";

export default function TrajectoryPlanningSetpoints({ trajectoryPlanningSetpoints, NUM_DRONES }: { trajectoryPlanningSetpoints: number[][], NUM_DRONES: number }) {
  const instancedMeshRef = useRef<InstancedMesh>()
  const temp = new Object3D()
  const tempColour = new Color()

  useEffect(() => {
    for (let droneIndex = 0; droneIndex < NUM_DRONES; droneIndex++) {
      trajectoryPlanningSetpoints.map(x => x.slice(droneIndex * 3, (droneIndex + 1) * 3)).forEach(([x, y, z]: Array<number>, i) => {
        temp.position.set(x, z, y) // y is up in threejs
        temp.updateMatrix()
        instancedMeshRef.current!.setMatrixAt(i + (droneIndex * trajectoryPlanningSetpoints.length), temp.matrix)
        instancedMeshRef.current!.setColorAt(i + (droneIndex * trajectoryPlanningSetpoints.length), tempColour.set(0x00ffff))
      })
    }

    instancedMeshRef.current!.instanceMatrix.needsUpdate = true
  }, [trajectoryPlanningSetpoints])

  return (
    <instancedMesh ref={instancedMeshRef} args={[undefined, undefined, trajectoryPlanningSetpoints.length * NUM_DRONES]}>
      <sphereGeometry args={[0.005, 4, 4]} />
      <meshLambertMaterial />
    </instancedMesh>
  )
}