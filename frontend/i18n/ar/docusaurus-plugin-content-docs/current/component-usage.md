---
sidebar_position: 2
---

# Component Usage Guide

This guide explains how to use the custom components available in the Physical AI & Humanoid Robotics platform.

## Core Components

### Button Component

The Button component comes in three variants: primary, secondary, and outline.

```jsx
import Button from '@site/src/components/core/Button';

<Button variant="primary">Primary Button</Button>
<Button variant="secondary">Secondary Button</Button>
<Button variant="outline">Outline Button</Button>
```

### MetricBadge Component

Display metrics with the MetricBadge component:

```jsx
import MetricBadge from '@site/src/components/core/MetricBadge';

<MetricBadge value="7" label="Chapters" variant="primary" />
<MetricBadge value="39" label="Lessons" variant="secondary" />
<MetricBadge value="100+" label="Exercises" variant="accent" />
```

### IconBubble Component

Create visually appealing icon containers:

```jsx
import IconBubble from '@site/src/components/core/IconBubble';

<IconBubble icon="🤖" size="md" color="neon-purple" />
```

## Layout Components

### SectionWrapper Component

Wrap content in styled sections:

```jsx
import SectionWrapper from '@site/src/components/layout/SectionWrapper';

<SectionWrapper background="gradient" padding="lg">
  <h2>Your Content Here</h2>
</SectionWrapper>
```

### GridWrapper Component

Create responsive grid layouts:

```jsx
import GridWrapper from '@site/src/components/layout/GridWrapper';

<GridWrapper cols={3} gap="lg">
  <div>Item 1</div>
  <div>Item 2</div>
  <div>Item 3</div>
</GridWrapper>
```

## Card Components

### FeatureCard Component

Display feature information with the FeatureCard component:

```jsx
import FeatureCard from '@site/src/components/core/FeatureCard';

<FeatureCard
  title="Feature Title"
  description="Feature description goes here."
  icon="🚀"
  badge="New"
/>
```

### SpectrumCard Component

Display AI spectrum information:

```jsx
import SpectrumCard from '@site/src/components/core/SpectrumCard';

<SpectrumCard
  type="assisted"
  title="AI Assisted"
  description="Human-guided systems where AI enhances human capabilities."
/>
```

### BookCard Component

Display book/course information:

```jsx
import BookCard from '@site/src/components/core/BookCard';

<BookCard
  title="Physical AI Fundamentals"
  author="Dr. Robotics"
  description="An introduction to embodied cognition and physical AI systems."
  chapters={7}
  lessons={39}
  exercises={100}
/>
```

## Section Components

### HeroSection Component

Create engaging hero sections for your pages:

```jsx
import HeroSection from '@site/src/components/sections/HeroSection';

<HeroSection />
```

### AISpectrumSection Component

Display the AI spectrum section:

```jsx
import AISpectrumSection from '@site/src/components/sections/AISpectrumSection';

<AISpectrumSection />
```

### FeatureGridSection Component

Show a grid of features:

```jsx
import FeatureGridSection from '@site/src/components/sections/FeatureGridSection';

<FeatureGridSection />
```

### CTAFooterSection Component

Add a call-to-action footer section:

```jsx
import CTAFooterSection from '@site/src/components/sections/CTAFooterSection';

<CTAFooterSection />
```

## Using Components in MDX

All components can be used directly in MDX files:

```mdx
---
title: My Page
---

import Button from '@site/src/components/core/Button';
import FeatureGridSection from '@site/src/components/sections/FeatureGridSection';

# My Page

<Button variant="primary">Click Me</Button>

<FeatureGridSection />
```