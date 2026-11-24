import fs from "fs"; import v from "gltf-validator";
const a=fs.readFileSync(process.argv[2]); const r=await v.validateBytes(new Uint8Array(a),{uri:"faibot-m1.glb"});
console.log(JSON.stringify({issues:r.issues, info:{v:r.info.version,gen:r.info.generator,drawCallCount:r.info.drawCallCount,materialCount:r.info.materialCount,totalVertexCount:r.info.totalVertexCount,totalTriangleCount:r.info.totalTriangleCount,maxAttributes:r.info.maxAttributes,hasAnimations:r.info.animationCount}},null,1));
