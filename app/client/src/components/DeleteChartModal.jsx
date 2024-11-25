import React, { useState } from 'react';
import '../styles/DeleteChartModal.css';

const DeleteChartModal = ({ onClose, onDeleteChart, charts }) => {
    const [selectedChart, setSelectedChart] = useState('');

    const handleDelete = () => {
        if (selectedChart) {

            console.log(`Attempting to remove chart: ${selectedChart} from localStorage...`);
            let storedCharts = JSON.parse(localStorage.getItem('charts')) || [];
            storedCharts = storedCharts.filter(chart => chart.label !== selectedChart);
            localStorage.setItem('charts', JSON.stringify(storedCharts));
            console.log(`Chart: ${selectedChart} successfully removed from localStorage.`);


            onDeleteChart(selectedChart);


            onClose();
        }
    };

    return (
        <div className="modal">
            <div className="modal-content">
                <h2>Select Chart to Delete</h2>

                {/* List of charts to select from */}
                <div className="form-group">
                    <label htmlFor="chartSelect">Choose Chart</label>
                    <select
                        id="chartSelect"
                        value={selectedChart}
                        onChange={(e) => setSelectedChart(e.target.value)}
                    >
                        <option value="">Select a chart</option>
                        {charts.map(chart => (
                            <option key={chart.id} value={chart.label}>
                                {chart.label}
                            </option>
                        ))}
                    </select>
                </div>

                <div className="modal-actions">
                    <button onClick={handleDelete}>Delete Chart</button>
                    <button onClick={onClose}>Cancel</button>
                </div>
            </div>
        </div>
    );
};

export default DeleteChartModal;
