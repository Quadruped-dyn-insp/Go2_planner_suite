/** @type {import('tailwindcss').Config} */
export default {
    content: [
        "./index.html",
        "./src/**/*.{js,ts,jsx,tsx}",
    ],
    theme: {
        extend: {
            colors: {
                'midnight': '#050A14',
                'deep-blue': '#0A1122',
                'neon-blue': '#00F0FF',
            },
            fontFamily: {
                sans: ['Inter', 'sans-serif'],
            },
            backgroundImage: {
                'radial-gradient': 'radial-gradient(circle at center, var(--tw-gradient-stops))',
            }
        },
    },
    plugins: [],
}
