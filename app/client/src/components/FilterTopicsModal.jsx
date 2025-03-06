/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */


import React, { useState, useEffect } from "react";
import Select from "react-select";
import CheckboxTree from "react-checkbox-tree";
import 'react-checkbox-tree/lib/react-checkbox-tree.css';
import "../styles/FilterTopicsModal.css";

const FilterTopicsModal = ({ isOpen, onClose, onApplyFilters }) => {
    const [filters, setFilters] = useState({
        name: "",
        messageTypes: [],
        namespaces: []
    });
    const handleApplyFilters = () => {
        const updatedFilters = {
            ...filters,
            namespaces: checkedNamespaces
        };

        onApplyFilters(updatedFilters);
        onClose();
    };

    const [namespaceTree, setNamespaceTree] = useState([]);
    const [checkedNamespaces, setCheckedNamespaces] = useState([]);
    const [expandedNamespaces, setExpandedNamespaces] = useState([]);
    const [dropdownOpen, setDropdownOpen] = useState(false);
    const [topicTypes, setTopicTypes] = useState([]);

    const handleCheck = (newChecked) => {
        let newSet = new Set(newChecked);

        const updateParentIfAnyChild = (node) => {
            if (!node.children || node.children.length === 0) {
                return;
            }

            const anyChildChecked = node.children.some((child) => newSet.has(child.value));

            if (anyChildChecked) {
                newSet.add(node.value);
            } else {
                if (!newChecked.includes(node.value)) {
                    newSet.delete(node.value);
                }
            }

            node.children.forEach((child) => updateParentIfAnyChild(child));
        };

        namespaceTree.forEach((rootNode) => updateParentIfAnyChild(rootNode));

        setCheckedNamespaces([...newSet]);
    };

    const convertToTree = (data, parentPath = "") => {
        return Object.entries(data).map(([key, value]) => {
            const fullPath = parentPath ? `${parentPath}/${key}` : `/${key}`;
            return {
                value: fullPath,
                label: key,
                children: Object.keys(value).length > 0 ? convertToTree(value, fullPath) : []
            };
        });
    };

    const messageTypeOptions = topicTypes.map(type => ({
        value: type,
        label: type
    }));

    useEffect(() => {
        if (isOpen) {
            const fetchNamespaces = async () => {
                try {
                    const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/namespaces`);
                    const data = await response.json();

                    const treeData = convertToTree(data);
                    setNamespaceTree(treeData);

                } catch (err) {
                    console.error("Error getting namespaces:", err);
                }
            };

            fetchNamespaces();
        }
    }, [isOpen]);

    useEffect(() => {
        if (isOpen) {
            const fetchTopicTypes = async () => {
                try {

                    const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/topic_types`);
                    const data = await response.json();
                    setTopicTypes(data);
                } catch (error) {
                    console.error('Error fetching topics:', error);
                }
            };

            fetchTopicTypes();
        }
    }, [isOpen]);

    if (!isOpen) return null;


    return (
        <div className="modal">
            <div className="modal-content filter-modal">
                <h2>Topics filter</h2>

                {/* Filter by name */}
                <div className="form-group">
                    <label>Topic name contains:</label>
                    <input
                        type="text"
                        value={filters.name}
                        onChange={(e) => setFilters({ ...filters, name: e.target.value })}
                        placeholder="Enter part of the topic name"
                    />
                </div>

                {/* Filtering by message type */}
                <div className="form-group">
                    <label>Message type:</label>
                    <Select
                        className="multi-select"
                        classNamePrefix="my-select"
                        isMulti
                        options={messageTypeOptions}
                        value={messageTypeOptions.filter(option => filters.messageTypes.includes(option.value))}
                        onChange={(selected) =>
                            setFilters({ ...filters, messageTypes: selected.map(option => option.value) })
                        }
                        placeholder="Select message type"
                        closeMenuOnSelect={false}
                    />
                </div>

                {/* Namespace filtering */}
                <div className="form-group">
                    <label>Namespace:</label>
                    <button
                        type="button"
                        className="namespace-toggle"
                        onClick={() => setDropdownOpen(!dropdownOpen)}
                    >
                        {checkedNamespaces.length > 0
                            ? `Selected: ${checkedNamespaces.length}`
                            : "Select namespace"}
                    </button>

                    {dropdownOpen && (
                        <div className="namespace-dropdown">
                            <CheckboxTree
                                noCascade
                                showPartialState={false}
                                nodes={namespaceTree}
                                checked={checkedNamespaces}
                                expanded={expandedNamespaces}
                                onCheck={handleCheck}
                                onExpand={setExpandedNamespaces}
                                showExpandAll
                                icons={{
                                    check: <span>✔</span>,
                                    uncheck: <span>◻</span>,
                                    halfCheck: <span>◧</span>,
                                    expandClose: <span>▶</span>,
                                    expandOpen: <span>▼</span>,
                                    parentClose: null,
                                    parentOpen: null,
                                    leaf: null
                                }}
                            />
                        </div>
                    )}
                </div>

                {/* Action buttons */}
                <div className="modal-actions">
                    <button onClick={onClose} className="cancel-button">Cancel</button>
                    <button onClick={
                        handleApplyFilters
                    } className="add-button">Apply Filters</button>
                </div>
            </div>
        </div>
    );
};

export default FilterTopicsModal;