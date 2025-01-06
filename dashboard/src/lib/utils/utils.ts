export function randomDarkColor(): string {
  const hue = Math.random() * 360;
  return `hsl(${hue}, 70%, 40%)`; // Saturated, darker colors
} 