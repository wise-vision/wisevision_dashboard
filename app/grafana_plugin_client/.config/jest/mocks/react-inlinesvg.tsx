/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Due to the grafana/ui Icon component making fetch requests to
// `/public/img/icon/<icon_name>.svg` we need to mock react-inlinesvg to prevent
// the failed fetch requests from displaying errors in console.

import React from 'react';

type Callback = (...args: any[]) => void;

export interface StorageItem {
  content: string;
  queue: Callback[];
  status: string;
}

export const cacheStore: { [key: string]: StorageItem } = Object.create(null);

const SVG_FILE_NAME_REGEX = /(.+)\/(.+)\.svg$/;

const InlineSVG = ({ src }: { src: string }) => {
  // testId will be the file name without extension (e.g. `public/img/icons/angle-double-down.svg` -> `angle-double-down`)
  const testId = src.replace(SVG_FILE_NAME_REGEX, '$2');
  return <svg xmlns="http://www.w3.org/2000/svg" data-testid={testId} viewBox="0 0 24 24" />;
};

export default InlineSVG;
