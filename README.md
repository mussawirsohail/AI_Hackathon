# Physical AI & Humanoid Robotics Book

This repository contains the source materials for the Physical AI & Humanoid Robotics book, implemented as a Docusaurus documentation site.

## Project Structure

```
docs/                   # Docusaurus documentation site
├── docs/               # Main book content
│   ├── module-1-the-robotic-nervous-system/
│   ├── module-2-the-digital-twin/
│   ├── module-3-the-ai-robot-brain/
│   ├── module-4-vision-language-action/
│   ├── assets/         # Diagrams and code examples
│   ├── intro.md
│   └── getting-started.md
├── src/                # Custom components and CSS
├── static/             # Static assets
├── docusaurus.config.ts # Site configuration
├── sidebars.ts         # Navigation structure
└── package.json        # Dependencies
specs/                  # Specification and planning documents
└── 001-physical-ai-book/ # Feature specifications
```

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Install dependencies**
   ```bash
   cd docs
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm run start
   # or
   yarn start
   ```
   This will start a local development server at `http://localhost:3000` with hot reloading enabled.

## Contributing

See `docs/README.md` for details on contributing to the book content.

## License

This educational content is provided under [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](https://creativecommons.org/licenses/by-nc-sa/4.0/).