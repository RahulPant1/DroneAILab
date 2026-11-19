// Text in the 3D view without fetching fonts (the app must work offline).

import { useEffect, useMemo } from 'react';
import * as THREE from 'three';

/** A text label drawn onto a small canvas and shown as a sprite (always faces the camera).
 *  Uses the system font, so it works offline. */
export default function Label({ text, color, position, size = 1 }: { text: string; color: string; position: [number, number, number]; size?: number }) {
  const texture = useMemo(() => {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d')!;
    ctx.font = 'bold 40px system-ui, sans-serif';
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 128, 32);
    const tex = new THREE.CanvasTexture(canvas);
    tex.colorSpace = THREE.SRGBColorSpace;
    return tex;
  }, [text, color]);
  useEffect(() => () => texture.dispose(), [texture]);
  return (
    <sprite position={position} scale={[0.6 * size, 0.15 * size, 1]}>
      <spriteMaterial map={texture} transparent depthWrite={false} />
    </sprite>
  );
}
