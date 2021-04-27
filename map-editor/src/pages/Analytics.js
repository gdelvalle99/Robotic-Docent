import {useState} from "react";
import Chart from "react-google-charts";
import { CircularProgress } from "@material-ui/core";
import ExhibitInfo from "../components/analytics/ExhibitInfo";
import DailyInfo from "../components/analytics/DailyInfo";
import RobotInfo from "../components/analytics/RobotInfo";
import FormDisplay from "../components/analytics/FormDisplay";

export const Analytics = () => {

    const [display, setDisplay] = useState("Exhibit Information");

    const handleChange = (event) => {
        console.log(event.target.value, display === "Exhibit Information")
        setDisplay(event.target.value);
    };

    return (
        <div
            style={{
                display: "flex",
                // alignItems: "center",
                // flexDirection: "column",
            }}
        >
            <div style={{ display: "flex", width: '100%' }}>
                {/* <Chart
                    width={400}
                    height={300}
                    chartType="ColumnChart"
                    loader={<CircularProgress />}
                    data={[
                        ["City", "2010 Population", "2000 Population"],
                        ["New York City, NY", 8175000, 8008000],
                        ["Los Angeles, CA", 3792000, 3694000],
                        ["Chicago, IL", 2695000, 2896000],
                        ["Houston, TX", 2099000, 1953000],
                        ["Philadelphia, PA", 1526000, 1517000],
                    ]}
                    options={{
                        title: "Population of Largest U.S. Cities",
                        chartArea: { width: "30%" },
                        hAxis: {
                            title: "Total Population",
                            minValue: 0,
                        },
                        vAxis: {
                            title: "City",
                        },
                    }}
                    legendToggle
                /> */}
                <FormDisplay display={display} handleChange={handleChange} style={{maxWidth: 1000, minWidth: 800}}/>
            </div>
            <div style={{ display: "flex", flexDirection: "column" }}>
                {display === "Exhibit Information" && <ExhibitInfo />}
                {display === "Robot Information" && <RobotInfo />}
                {display === "Daily Information" && <DailyInfo />}
            </div>
        </div>
    );
};
