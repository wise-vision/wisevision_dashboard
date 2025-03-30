/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React from 'react';
import PropTypes from 'prop-types';
import './Card.css';

/**
 * Card component for dashboard items
 */
const Card = ({
  children,
  title,
  subtitle,
  actions,
  elevation = 'md',
  className = '',
  isLoading = false,
  isDraggable = false,
  onDragStart,
  ...props
}) => {
  const cardClasses = [
    'card',
    `card-elevation-${elevation}`,
    isDraggable ? 'card-draggable' : '',
    className
  ].filter(Boolean).join(' ');

  return (
    <div 
      className={cardClasses}
      draggable={isDraggable}
      onDragStart={isDraggable ? onDragStart : undefined}
      {...props}
    >
      {isLoading && <div className="card-loader" />}
      
      {(title || actions) && (
        <div className="card-header">
          <div className="card-header-left">
            {title && <h3 className="card-title">{title}</h3>}
            {subtitle && <p className="card-subtitle">{subtitle}</p>}
          </div>
          {actions && <div className="card-actions">{actions}</div>}
        </div>
      )}
      
      <div className={`card-content ${isLoading ? 'card-content-loading' : ''}`}>
        {children}
      </div>
    </div>
  );
};

Card.propTypes = {
  children: PropTypes.node,
  title: PropTypes.node,
  subtitle: PropTypes.node,
  actions: PropTypes.node,
  elevation: PropTypes.oneOf(['sm', 'md', 'lg', 'xl']),
  className: PropTypes.string,
  isLoading: PropTypes.bool,
  isDraggable: PropTypes.bool,
  onDragStart: PropTypes.func
};

export default Card;
