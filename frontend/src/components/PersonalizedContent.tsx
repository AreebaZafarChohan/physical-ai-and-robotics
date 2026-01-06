<<<<<<< HEAD
import React, { useEffect, useRef, useState } from 'react';
import '../css/personalization.css';

interface PersonalizedContentProps {
  content: string;
  highlightPersonalized?: boolean; // Whether to highlight personalized sections
  className?: string;
  showBadge?: boolean; // Show "Personalized for You" badge
}

const PersonalizedContent: React.FC<PersonalizedContentProps> = ({
  content,
  highlightPersonalized = true,
  className = '',
  showBadge = true
}) => {
  const contentRef = useRef<HTMLDivElement>(null);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    // Trigger animation after mount
    setTimeout(() => setIsVisible(true), 100);

    // Update the content when it changes
    if (contentRef.current) {
      // Set the content as HTML
      contentRef.current.innerHTML = content;

      // Highlight personalized sections if requested
      if (highlightPersonalized) {
        addHighlighting();
      }

      // Run any additional processing after content is inserted
      const scripts = contentRef.current.querySelectorAll('script');
      scripts.forEach(oldScript => {
        const newScript = document.createElement('script');
        if (oldScript.src) {
          newScript.src = oldScript.src;
        } else {
          newScript.textContent = oldScript.textContent;
        }
        oldScript.parentNode?.replaceChild(newScript, oldScript);
      });
    }
  }, [content, highlightPersonalized]);

  const addHighlighting = () => {
    if (!contentRef.current) return;

    // Add visual highlighting to different elements

    // Highlight headings with purple underline effect
    const headings = contentRef.current.querySelectorAll('h1, h2, h3, h4, h5, h6');
    headings.forEach((heading, index) => {
      heading.classList.add('personalized-underline');
      // Stagger animation
      (heading as HTMLElement).style.animationDelay = `${index * 0.1}s`;
    });

    // Add highlighter effect to important paragraphs (first paragraph, and paragraphs with strong tags)
    const paragraphs = contentRef.current.querySelectorAll('p');
    paragraphs.forEach((p, index) => {
      if (index === 0 || p.querySelector('strong')) {
        // Add subtle highlight background to first paragraph
        p.classList.add('highlight-purple');
      }
    });

    // Highlight code blocks
    const codeBlocks = contentRef.current.querySelectorAll('pre');
    codeBlocks.forEach(pre => {
      const wrapper = document.createElement('div');
      wrapper.className = 'personalized-code-wrapper';
      pre.parentNode?.insertBefore(wrapper, pre);
      wrapper.appendChild(pre);
    });

    // Highlight inline code with yellow marker
    const inlineCodes = contentRef.current.querySelectorAll('code:not(pre code)');
    inlineCodes.forEach(code => {
      code.classList.add('highlight-yellow');
    });

    // Highlight blockquotes
    const blockquotes = contentRef.current.querySelectorAll('blockquote');
    blockquotes.forEach(bq => {
      bq.classList.add('personalized-callout');
    });

    // Add reveal animation to list items
    const listItems = contentRef.current.querySelectorAll('li');
    listItems.forEach((li, index) => {
      li.classList.add('personalized-reveal');
      // Trigger visibility with slight delay
      setTimeout(() => li.classList.add('visible'), 100 + (index * 50));
    });

    // Highlight links
    const links = contentRef.current.querySelectorAll('a');
    links.forEach(link => {
      link.classList.add('personalized-text-highlight');
    });

    // Add special highlighting to strong/bold text
    const strongTexts = contentRef.current.querySelectorAll('strong, b');
    strongTexts.forEach(strong => {
      strong.classList.add('highlight-green');
    });

    // Highlight tables if any
    const tables = contentRef.current.querySelectorAll('table');
    tables.forEach(table => {
      const wrapper = document.createElement('div');
      wrapper.className = 'personalized-table-wrapper highlight-blue';
      table.parentNode?.insertBefore(wrapper, table);
      wrapper.appendChild(table);
    });
  };

  return (
    <div className={`personalized-content-block ${isVisible ? 'visible' : ''} ${className}`}>
      {/* Personalized Badge */}
      {showBadge && (
        <div className="personalized-badge">
          Personalized for You
        </div>
      )}

      {/* Section Header */}
      <div className="personalized-section-header">
        <svg className="icon" viewBox="0 0 24 24" fill="currentColor">
          <path d="M12 2L15.09 8.26L22 9.27L17 14.14L18.18 21.02L12 17.77L5.82 21.02L7 14.14L2 9.27L8.91 8.26L12 2Z"/>
        </svg>
        <h3>Content Tailored to Your Profile</h3>
      </div>

      {/* Main Content */}
      <div
        ref={contentRef}
        className={`personalized-content fresh ${className}`}
      />

      {/* Footer hint */}
      <div className="personalized-footer-hint">
        <span className="hint-icon">💡</span>
        <span className="hint-text">This content has been personalized based on your learning profile and preferences.</span>
      </div>
    </div>
  );
};

export default PersonalizedContent;
=======
import React, { useEffect, useRef, useState } from 'react';
import '../css/personalization.css';

interface PersonalizedContentProps {
  content: string;
  highlightPersonalized?: boolean; // Whether to highlight personalized sections
  className?: string;
  showBadge?: boolean; // Show "Personalized for You" badge
}

const PersonalizedContent: React.FC<PersonalizedContentProps> = ({
  content,
  highlightPersonalized = true,
  className = '',
  showBadge = true
}) => {
  const contentRef = useRef<HTMLDivElement>(null);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    // Trigger animation after mount
    setTimeout(() => setIsVisible(true), 100);

    // Update the content when it changes
    if (contentRef.current) {
      // Set the content as HTML
      contentRef.current.innerHTML = content;

      // Highlight personalized sections if requested
      if (highlightPersonalized) {
        addHighlighting();
      }

      // Run any additional processing after content is inserted
      const scripts = contentRef.current.querySelectorAll('script');
      scripts.forEach(oldScript => {
        const newScript = document.createElement('script');
        if (oldScript.src) {
          newScript.src = oldScript.src;
        } else {
          newScript.textContent = oldScript.textContent;
        }
        oldScript.parentNode?.replaceChild(newScript, oldScript);
      });
    }
  }, [content, highlightPersonalized]);

  const addHighlighting = () => {
    if (!contentRef.current) return;

    // Add visual highlighting to different elements

    // Highlight headings with purple underline effect
    const headings = contentRef.current.querySelectorAll('h1, h2, h3, h4, h5, h6');
    headings.forEach((heading, index) => {
      heading.classList.add('personalized-underline');
      // Stagger animation
      (heading as HTMLElement).style.animationDelay = `${index * 0.1}s`;
    });

    // Add highlighter effect to important paragraphs (first paragraph, and paragraphs with strong tags)
    const paragraphs = contentRef.current.querySelectorAll('p');
    paragraphs.forEach((p, index) => {
      if (index === 0 || p.querySelector('strong')) {
        // Add subtle highlight background to first paragraph
        p.classList.add('highlight-purple');
      }
    });

    // Highlight code blocks
    const codeBlocks = contentRef.current.querySelectorAll('pre');
    codeBlocks.forEach(pre => {
      const wrapper = document.createElement('div');
      wrapper.className = 'personalized-code-wrapper';
      pre.parentNode?.insertBefore(wrapper, pre);
      wrapper.appendChild(pre);
    });

    // Highlight inline code with yellow marker
    const inlineCodes = contentRef.current.querySelectorAll('code:not(pre code)');
    inlineCodes.forEach(code => {
      code.classList.add('highlight-yellow');
    });

    // Highlight blockquotes
    const blockquotes = contentRef.current.querySelectorAll('blockquote');
    blockquotes.forEach(bq => {
      bq.classList.add('personalized-callout');
    });

    // Add reveal animation to list items
    const listItems = contentRef.current.querySelectorAll('li');
    listItems.forEach((li, index) => {
      li.classList.add('personalized-reveal');
      // Trigger visibility with slight delay
      setTimeout(() => li.classList.add('visible'), 100 + (index * 50));
    });

    // Highlight links
    const links = contentRef.current.querySelectorAll('a');
    links.forEach(link => {
      link.classList.add('personalized-text-highlight');
    });

    // Add special highlighting to strong/bold text
    const strongTexts = contentRef.current.querySelectorAll('strong, b');
    strongTexts.forEach(strong => {
      strong.classList.add('highlight-green');
    });

    // Highlight tables if any
    const tables = contentRef.current.querySelectorAll('table');
    tables.forEach(table => {
      const wrapper = document.createElement('div');
      wrapper.className = 'personalized-table-wrapper highlight-blue';
      table.parentNode?.insertBefore(wrapper, table);
      wrapper.appendChild(table);
    });
  };

  return (
    <div className={`personalized-content-block ${isVisible ? 'visible' : ''} ${className}`}>
      {/* Personalized Badge */}
      {showBadge && (
        <div className="personalized-badge">
          Personalized for You
        </div>
      )}

      {/* Section Header */}
      <div className="personalized-section-header">
        <svg className="icon" viewBox="0 0 24 24" fill="currentColor">
          <path d="M12 2L15.09 8.26L22 9.27L17 14.14L18.18 21.02L12 17.77L5.82 21.02L7 14.14L2 9.27L8.91 8.26L12 2Z"/>
        </svg>
        <h3>Content Tailored to Your Profile</h3>
      </div>

      {/* Main Content */}
      <div
        ref={contentRef}
        className={`personalized-content fresh ${className}`}
      />

      {/* Footer hint */}
      <div className="personalized-footer-hint">
        <span className="hint-icon">💡</span>
        <span className="hint-text">This content has been personalized based on your learning profile and preferences.</span>
      </div>
    </div>
  );
};

export default PersonalizedContent;
>>>>>>> 012-docusaurus-i18n-urdu
