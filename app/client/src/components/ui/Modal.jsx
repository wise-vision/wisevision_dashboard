/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useEffect, useRef } from 'react';
import PropTypes from 'prop-types';
import './Modal.css';

/**
 * Modal component with animation and backdrop
 */
const Modal = ({
  isOpen,
  onClose,
  children,
  title,
  size = 'md',
  showCloseButton = true,
  closeOnBackdropClick = true,
  closeOnEscape = true,
  animation = 'scale',
  className = '',
  contentClassName = '',
  testId = 'modal',
}) => {
  const modalRef = useRef(null);

  useEffect(() => {
    const handleEscape = (e) => {
      if (closeOnEscape && isOpen && e.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('keydown', handleEscape);
    
    // Prevent body scrolling when modal is open
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    }
    
    return () => {
      document.removeEventListener('keydown', handleEscape);
      document.body.style.overflow = '';
    };
  }, [isOpen, closeOnEscape, onClose]);

  if (!isOpen) return null;

  const handleBackdropClick = (e) => {
    if (closeOnBackdropClick && e.target === e.currentTarget) {
      onClose();
    }
  };

  const modalClasses = [
    'modal-overlay',
    `modal-animation-${animation}`,
    isOpen ? 'modal-open' : '',
    className,
  ].filter(Boolean).join(' ');

  const contentClasses = [
    'modal-content',
    `modal-${size}`,
    contentClassName,
  ].filter(Boolean).join(' ');

  return (
    <div 
      className={modalClasses}
      onClick={handleBackdropClick}
      data-testid={testId}
      role="dialog"
      aria-modal="true"
    >
      <div className={contentClasses} ref={modalRef}>
        {(title || showCloseButton) && (
          <div className="modal-header">
            {title && <h2 className="modal-title">{title}</h2>}
            {showCloseButton && (
              <button 
                className="modal-close"
                onClick={onClose}
                aria-label="Close modal"
              >
                ×
              </button>
            )}
          </div>
        )}
        <div className="modal-body">
          {children}
        </div>
      </div>
    </div>
  );
};

Modal.propTypes = {
  isOpen: PropTypes.bool.isRequired,
  onClose: PropTypes.func.isRequired,
  children: PropTypes.node.isRequired,
  title: PropTypes.node,
  size: PropTypes.oneOf(['sm', 'md', 'lg', 'xl', 'full']),
  showCloseButton: PropTypes.bool,
  closeOnBackdropClick: PropTypes.bool,
  closeOnEscape: PropTypes.bool,
  animation: PropTypes.oneOf(['scale', 'slide-up', 'fade', 'none']),
  className: PropTypes.string,
  contentClassName: PropTypes.string,
  testId: PropTypes.string,
};

export default Modal;
