# Ripple Brand Colors (from icon.png analysis)

## Primary Colors
- **Cyan**: #00D4FF (bright cyan/turquoise)
- **Purple**: #9B7BFF (lavender/violet)
- **Green**: #00E676 (bright mint green)

## Background
- **Deep Black**: #0A0A0A or #000000

## Gradient Direction
- The spiral flows from cyan → purple → green
- Creates a dynamic, flowing effect

## Theme Configuration (from theme.config.js)
```javascript
primary: '#00D4FF',    // Cyan
secondary: '#9B7BFF',  // Purple  
accent: '#00E676',     // Green
background: '#0A0A0A', // Deep black
surface: '#1A1A1A',    // Dark surface
foreground: '#ECEDEE', // Light text
muted: '#9BA1A6',      // Muted text
border: '#2A2A2A',     // Border color
```

## Usage
- Dark theme is primary (matching the black background of the icon)
- Gradient text/borders use all three colors
- Primary actions use cyan
- Secondary elements use purple
- Success/accent states use green
