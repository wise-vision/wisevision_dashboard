import React, { useState, useEffect } from 'react';
import jsPDF from 'jspdf';
import '../styles/CreateReportModal.css';

const CreateReportModal = ({ isOpen, onClose }) => {
    const [topics, setTopics] = useState([]);
    const [selectedTopic, setSelectedTopic] = useState('');
    const [reportName, setReportName] = useState('');
    const [errorMessage, setErrorMessage] = useState(''); // Dodano dla obsługi błędów

    useEffect(() => {
        if (isOpen) {
            console.log('Modal is open, fetching topics...');
            fetchTopics();
        }
    }, [isOpen]);

    // Fetch list of topics from backend
    const fetchTopics = async () => {
        try {
            console.log('Fetching topics from API...');
            const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/topics`);
            const data = await response.json();
            console.log('Topics fetched successfully:', data);
            setTopics(data);
        } catch (error) {
            console.error('Error fetching topics:', error);
        }
    };

    const handleGenerateReport = async () => {
        try {
            setErrorMessage(''); // Resetujemy komunikat błędu na początku
            if (!selectedTopic) {
                setErrorMessage('Please select a topic.');
                return;
            }

            console.log(`Starting report generation for topic: ${selectedTopic}`);
            const encodedTopic = encodeURIComponent(selectedTopic);
            const response = await fetch(
                `${process.env.REACT_APP_API_BASE_URL}/api/topic_echo_data_base_any/${encodedTopic}?type=lora_msgs/msg/E5BoardUplink`
            );
            const data = await response.json();

            const messages = data.messages || [];
            if (messages.length === 0) {
                setErrorMessage('Not available'); // Ustawiamy komunikat, jeśli brak danych
                console.log('No data available for the selected topic.');
                return;
            }

            // Create a new PDF document
            const doc = new jsPDF();
            console.log('New PDF document created.');

            let yOffset = 20;
            const lineHeight = 10;
            const pageWidth = doc.internal.pageSize.getWidth();
            const margin = 10;
            const usableWidth = pageWidth - margin * 2;

            doc.setFontSize(8);

            // Add topic header
            doc.text(`Topic: ${selectedTopic}`, margin, yOffset);
            yOffset += lineHeight;

            messages.forEach((message, index) => {
                const messageText = `Message ${index + 1}: ${JSON.stringify(message)}`;
                const splitText = doc.splitTextToSize(messageText, usableWidth);

                splitText.forEach((line) => {
                    if (yOffset > doc.internal.pageSize.getHeight() - margin) {
                        console.log('Content exceeds page height, adding new page...');
                        doc.addPage();
                        yOffset = margin;
                    }
                    doc.text(line, margin, yOffset);
                    yOffset += lineHeight;
                });
            });

            const fileName = reportName.trim() ? `${reportName}.pdf` : 'report.pdf';
            console.log(`Saving PDF document as: ${fileName}`);

            // Save the generated PDF
            doc.save(fileName);
            console.log('PDF document saved successfully.');

            onClose();
        } catch (error) {
            console.error('Error generating report:', error);
            setErrorMessage('Error generating report. Please try again.');
        }
    };

    if (!isOpen) return null;

    return (
        <div className="modal">
            <div className="modal-content">
                <h2>Generate Report</h2>

                {/* Report Name */}
                <div className="form-group">
                    <label>Report Name:</label>
                    <input
                        type="text"
                        value={reportName}
                        onChange={(e) => setReportName(e.target.value)}
                        placeholder="Enter report name"
                    />
                </div>

                {/* Dropdown Topic Selection */}
                <div className="form-group">
                    <label>Select Topic:</label>
                    <select
                        value={selectedTopic}
                        onChange={(e) => setSelectedTopic(e.target.value)}
                    >
                        <option value="" disabled>Select a topic</option>
                        {topics.map((topic) => (
                            <option key={topic.id} value={topic.name}>
                                {topic.name}
                            </option>
                        ))}
                    </select>
                </div>

                {/* Error Message */}
                {errorMessage && <p className="error-message">{errorMessage}</p>}

                {/* Actions */}
                <div className="modal-actions">
                    <button onClick={handleGenerateReport}>Generate Report</button>
                    <button onClick={onClose}>Cancel</button>
                </div>
            </div>
        </div>
    );
};

export default CreateReportModal;
