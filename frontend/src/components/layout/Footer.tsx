import React from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';

type FooterLink = {
  label: string;
  to?: string;
  href?: string;
};

type FooterColumn = {
  title: string;
  items: FooterLink[];
};

const Footer = () => {
  const footerColumns: FooterColumn[] = [
    {
      title: 'Docs',
      items: [
        {
          label: 'Introduction',
          to: '/docs/intro',
        },
        {
          label: 'ROS 2 Guide',
          to: '/docs/category/ros-2-fundamentals',
        },
        {
          label: 'Digital Twin',
          to: '/docs/category/digital-twin-concepts',
        },
        {
          label: 'Vision-Language-Action (VLA)',
          to: '/docs/category/vla-models',
        },
      ],
    },
    {
      title: 'Community',
      items: [
        {
          label: 'GitHub Discussions',
          href: 'https://github.com/AreebaZafarChohan/physical-ai-and-robotics/discussions',
        },
        {
          label: 'Discord',
          href: 'https://discord.gg/robotics',
        },
        {
          label: 'Robotics Stack Exchange',
          href: 'https://robotics.stackexchange.com/',
        },
        {
          label: 'ROS Answers',
          href: 'https://answers.ros.org/questions/',
        },
      ],
    },
    {
      title: 'Resources',
      items: [
        {
          label: 'Tutorials',
          to: '/docs/tutorials',
        },
        {
          label: 'Research Papers',
          to: '/papers',
        },
        {
          label: 'Videos',
          to: '/videos',
        },
        {
          label: 'Blog',
          to: '/blog',
        },
      ],
    },
    {
      title: 'More',
      items: [
        {
          label: 'About Us',
          to: '/about',
        },
        {
          label: 'Team',
          to: '/team',
        },
        {
          label: 'Contact',
          to: '/contact',
        },
        {
          label: 'GitHub',
          href: 'https://github.com/AreebaZafarChohan/physical-ai-and-robotics',
        },
      ],
    },
  ];

  return (
    <footer className="footer">
      <div className="container">
        {/* Newsletter Section */}
        <div className="footer__newsletter">
          <h3>Join Our Robotics Community</h3>
          <p>Subscribe to get the latest updates on AI and robotics research</p>
          <div className="footer__newsletter-input">
            <input type="email" placeholder="Enter your email" />
            <button>Subscribe</button>
          </div>
        </div>

        {/* Footer Links */}
        <div className="footer__links">
          {footerColumns.map((column, index) => (
            <div key={index} className="footer__col">
              <h4 className="footer__title">{column.title}</h4>
              <ul className="footer__items">
                {column.items.map((item, itemIndex) => (
                  <li key={itemIndex} className="footer__item">
                    {item.to ? (
                      <Link to={item.to} className="footer__link">
                        {item.label}
                      </Link>
                    ) : (
                      <a
                        href={item.href}
                        className="footer__link"
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        {item.label}
                      </a>
                    )}
                  </li>
                ))}
              </ul>
            </div>
          ))}
        </div>

        {/* Bottom Section */}
        <div className="footer__bottom">
          <div className="footer__copyright">
            <p>
              Copyright © {new Date().getFullYear()}{' '}
              <Link to="/">RoboX - An Open-Source Robotics Initiative</Link>.
            </p>
            <p>
              Built with <Link to="https://docusaurus.io/">Docusaurus</Link>
            </p>
            
            <div className="footer__legal-links">
              <Link to="/privacy">Privacy Policy</Link> •{' '}
              <Link to="/terms">Terms of Service</Link> •{' '}
              <Link to="/cookies">Cookie Policy</Link>
            </div>
          </div>
        </div>
      </div>
    </footer>
  );
};

export default Footer;