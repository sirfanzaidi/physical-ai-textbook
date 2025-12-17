/**
 * Citation Component
 *
 * Displays a single citation with chapter info and link to source.
 */

import React from 'react';
import { Citation as CitationType } from '../../services/apiClient';
import styles from './RAGChat.module.css';

interface CitationProps {
  citation: CitationType;
}

export const Citation: React.FC<CitationProps> = ({ citation }) => {
  return (
    <a
      href={citation.source_url}
      target="_blank"
      rel="noopener noreferrer"
      className={styles.citationLink}
      title={`${citation.chapter_name}${citation.section_name ? ` - ${citation.section_name}` : ''}`}
    >
      <div className={styles.citationContent}>
        <div className={styles.citationChapter}>{citation.chapter_name}</div>
        {citation.section_name && (
          <div className={styles.citationSection}>{citation.section_name}</div>
        )}
        {citation.page_num && (
          <div className={styles.citationPage}>p. {citation.page_num}</div>
        )}
      </div>
      <div className={styles.citationIcon}>→</div>
    </a>
  );
};
