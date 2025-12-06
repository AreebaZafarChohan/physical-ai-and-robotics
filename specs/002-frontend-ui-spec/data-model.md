# Data Model: Frontend UI for Physical AI & Humanoid Robotics Platform

## Component Data Structures

### Button Component
- **Props:**
  - variant: 'primary' | 'secondary' | 'outline'
  - size: 'sm' | 'md' | 'lg'
  - children: ReactNode
  - onClick: () => void
  - disabled: boolean
  - className: string (optional)

### MetricBadge Component
- **Props:**
  - value: number | string
  - label: string
  - icon: ReactNode (optional)
  - className: string (optional)

### FeatureCard Component
- **Props:**
  - title: string
  - description: string
  - icon: ReactNode
  - link: string (optional)
  - className: string (optional)

### SpectrumCard Component
- **Props:**
  - level: 'assisted' | 'driven' | 'native'
  - title: string
  - description: string
  - icon: ReactNode
  - className: string (optional)

### BookCard Component
- **Props:**
  - title: string
  - description: string
  - coverImage: ReactNode
  - topics: string[]
  - className: string (optional)

### Navbar Component
- **Props:**
  - items: Array<{label: string, href: string}>
  - logo: ReactNode
  - sticky: boolean
  - className: string (optional)

### Footer Component
- **Props:**
  - links: Array<{text: string, href: string}>
  - socialLinks: Array<{platform: string, url: string}>
  - copyrightText: string
  - className: string (optional)

### HeroSection Component
- **Props:**
  - title: string
  - subtitle: string
  - ctaButtons: Array<{text: string, variant: 'primary' | 'secondary'}>
  - metrics: {chapters: number, lessons: number, exercises: number}
  - illustration: ReactNode
  - className: string (optional)

### SectionWrapper Component
- **Props:**
  - children: ReactNode
  - className: string (optional)
  - padded: boolean (default: true)
  - dark: boolean (default: false)

### GridWrapper Component
- **Props:**
  - children: ReactNode
  - columns: {base: number, md: number, lg: number}
  - gap: 'sm' | 'md' | 'lg'
  - className: string (optional)

### ContentSection Component
- **Props:**
  - title: string
  - subtitle: string (optional)
  - children: ReactNode
  - centered: boolean (default: false)
  - className: string (optional)

### CTASection Component
- **Props:**
  - title: string
  - subtitle: string
  - buttons: Array<{text: string, variant: 'primary' | 'secondary'}>
  - background: 'gradient' | 'solid' (default: 'gradient')
  - className: string (optional)

## Theme Configuration Data

### Color Palette
- Primary Purple: #8A2BE2 (Deep, vibrant purple)
- Secondary Purple: #9370DB (Medium purple for secondary elements)
- Accent Neon: #00FFFF (Cyan for highlights and interactive elements)
- Background Dark: #0F0F14 (Dark background for most pages)
- Surface Dark: #1A1A20 (Slightly lighter for cards and containers)
- Text Primary: #FFFFFF (White for primary text)
- Text Secondary: #B0B0B0 (Gray for secondary text)
- Success: #00FF00 (Green for positive feedback)
- Warning: #FFFF00 (Yellow for warnings)
- Error: #FF0000 (Red for errors)

### Typography Scale
- Heading 1 (H1): 3rem (48px) / 48px line height
- Heading 2 (H2): 2.5rem (40px) / 40px line height
- Heading 3 (H3): 2rem (32px) / 32px line height
- Heading 4 (H4): 1.75rem (28px) / 28px line height
- Heading 5 (H5): 1.5rem (24px) / 24px line height
- Heading 6 (H6): 1.25rem (20px) / 24px line height
- Body Large: 1.125rem (18px) / 28px line height
- Body: 1rem (16px) / 24px line height
- Body Small: 0.875rem (14px) / 20px line height
- Caption: 0.75rem (12px) / 16px line height

### Spacing Scale
- Spacing 0: 0px
- Spacing 1: 4px
- Spacing 2: 8px
- Spacing 3: 12px
- Spacing 4: 16px
- Spacing 5: 20px
- Spacing 6: 24px
- Spacing 7: 32px
- Spacing 8: 40px
- Spacing 9: 48px
- Spacing 10: 64px
- Spacing 11: 80px
- Spacing 12: 96px

### Breakpoints
- xs: 0px
- sm: 640px
- md: 768px
- lg: 1024px
- xl: 1280px
- 2xl: 1536px