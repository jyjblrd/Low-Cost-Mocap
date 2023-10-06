export const numberToHexColor = (number: number, n: number): number => {
    // Ensure the number is within the range 0 to n
    number = Math.min(Math.max(number, 0), n);

    // Calculate the value for each color channel (R, G, B)
    const red = Math.floor((number / n) * 255);
    const green = Math.floor(((n - number) / n) * 255);
    const blue = 0;

    // Convert the RGB values to hexadecimal and combine them into a single number
    const hexColor = (red << 16) + (green << 8) + blue;

    return hexColor;
};