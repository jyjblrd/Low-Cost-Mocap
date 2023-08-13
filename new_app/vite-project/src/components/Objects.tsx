import { MutableRefObject, useEffect, useRef } from "react";
import { BufferAttribute, BufferGeometry, EdgesGeometry, InstancedMesh, LineBasicMaterial, Matrix4, Object3D } from "three";

export default function Objects({objectsRef, count}: {objectsRef: MutableRefObject<Object[][]>, count: number}) {
  let objects = objectsRef.current.flat()
  objects = objects.length == 0 ? [] : [objects[objects.length-1]]

  const instancedMeshRef = useRef<InstancedMesh>()
  const temp = new Object3D()

  useEffect(() => {
    objects.forEach(({location, rotationMatrix, error}, i) => {
      console.log(location)
      temp.position.set(location[0], location[1], location[2])
      let threeRotationMatrix = new Matrix4
      threeRotationMatrix.set(
        rotationMatrix[0][0], rotationMatrix[0][1], rotationMatrix[0][2], 0,
        rotationMatrix[1][0], rotationMatrix[1][1], rotationMatrix[1][2], 0,
        rotationMatrix[2][0], rotationMatrix[2][1], rotationMatrix[2][2], 0,
        0, 0, 0, 1
      )
      temp.setRotationFromMatrix(threeRotationMatrix)
      temp.updateMatrix()
      instancedMeshRef.current!.setMatrixAt(i, temp.matrix)
    })
    instancedMeshRef.current!.instanceMatrix.needsUpdate = true
  }, [count])
  return (
    <instancedMesh ref={instancedMeshRef} args={[undefined, undefined, objects.length]}>
        <boxGeometry args={[0.1,0.05,0.1]}/>
        <meshPhongMaterial/>
    </instancedMesh>
  )
}