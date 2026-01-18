import React, { useState, useEffect } from 'react';
import { useLocation, useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { translate } from '@docusaurus/Translate';

interface Language {
  locale: string;
  label: string;
  direction: 'ltr' | 'rtl';
}

const LANGUAGES: Language[] = [
  { locale: 'en', label: 'English', direction: 'ltr' },
  { locale: 'ur', label: 'اردو', direction: 'rtl' },
  { locale: 'ar', label: 'العربية', direction: 'rtl' },
];

const LanguageSwitcher: React.FC = () => {
  const { i18n } = useDocusaurusContext();
  const { currentLocale } = i18n;
  const [selectedLocale, setSelectedLocale] = useState(currentLocale);
  const history = useHistory();
  const location = useLocation();
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);

  // Update the HTML direction attribute when locale changes
  useEffect(() => {
    const htmlElement = document.documentElement;
    const currentLang = LANGUAGES.find(lang => lang.locale === selectedLocale);
    if (currentLang) {
      htmlElement.setAttribute('dir', currentLang.direction);
      htmlElement.setAttribute('lang', currentLang.locale);
    }
  }, [selectedLocale]);

  const handleLocaleChange = (newLocale: string) => {
    // Update the selected locale
    setSelectedLocale(newLocale);

    // Get all locale codes to filter them out
    const localeCodes = LANGUAGES.map(lang => lang.locale);

    // Remove ALL locale segments from the path
    let pathSegments = location.pathname.split('/').filter(segment =>
      segment && !localeCodes.includes(segment)
    );

    // Build the new path with the selected locale
    const newPath = newLocale === 'en'
      ? `/${pathSegments.join('/')}`  // English is the default, no locale prefix needed
      : `/${newLocale}/${pathSegments.join('/')}`;

    // Navigate to the new locale path
    history.push(newPath);

    // Close the dropdown after selection
    setIsDropdownOpen(false);
  };

  const currentLanguage = LANGUAGES.find(lang => lang.locale === selectedLocale);

  return (
    <div className="language-switcher relative">
      <button
        onClick={() => setIsDropdownOpen(!isDropdownOpen)}
        className="flex items-center justify-center px-3 py-2 text-sm font-medium text-gray-700 dark:text-gray-200 bg-gray-100 dark:bg-gray-700 rounded-md hover:bg-gray-200 dark:hover:bg-gray-600 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500"
        aria-label={translate({ id: 'navbar.language_switcher.aria_label', message: 'Language switcher' })}
      >
        <span className="mr-2">{currentLanguage?.label}</span>
        <svg 
          className={`w-4 h-4 transition-transform duration-200 ${isDropdownOpen ? 'rotate-180' : ''}`} 
          fill="none" 
          stroke="currentColor" 
          viewBox="0 0 24 24" 
          xmlns="http://www.w3.org/2000/svg"
        >
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
        </svg>
      </button>

      {isDropdownOpen && (
        <div className="origin-top-right absolute right-0 mt-2 w-40 rounded-md shadow-lg bg-white dark:bg-gray-800 ring-1 ring-black ring-opacity-5 z-50">
          <div className="py-1" role="menu">
            {LANGUAGES.filter(lang => lang.locale !== selectedLocale).map((lang) => (
              <button
                key={lang.locale}
                onClick={() => handleLocaleChange(lang.locale)}
                className="block px-4 py-2 text-sm text-gray-700 dark:text-gray-200 hover:bg-gray-100 dark:hover:bg-gray-700 w-full text-left"
                role="menuitem"
              >
                {lang.label}
              </button>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default LanguageSwitcher;