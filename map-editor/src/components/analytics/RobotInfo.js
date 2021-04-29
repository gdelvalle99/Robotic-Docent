import React, { useState, useEffect } from "react";
import Chart from "react-google-charts";
import { CircularProgress } from "@material-ui/core";
import { mapLink, floor_id } from "../../links";

export default function RobotInfo() {
    const [image, setImage] = useState("");

    const getImage = () => {
        const formData = new FormData();
        formData.append("floor_id", floor_id); // just for testing

        const token = localStorage.getItem("auth_token") || "";
        fetch(mapLink, {
            method: "POST",
            headers: {
                Accept: "application/json, text/plain, */*",
                Authentification: token,
                credentials: true,
            },
            body: formData,
        })
            .then((response) => {
                return response.blob();
            })
            .then((blob) => {
                const link = URL.createObjectURL(blob);
                setImage(link);
            })
            .catch((e) => console.log(e));
    };
    useEffect(() => {
        getImage();
    }, []);

    return (
        <div style={{ display: "flex", maxWidth: 500, flexDirection: "column" }}>
            <div className="img-overlay-wrap">
                {image && <img src={image} className="map-image" alt="iwi" width={300}/>}
                <svg id="map" className="exhibits" viewBox="0 0 200 250">
                    <defs>
                        <symbol id="pin" className="millet" viewBox="0 0 24 24">
                            <path
                                id="thePath"
                                d="M12 2c-3.87 0-7 3.13-7 7 0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z"
                            ></path>
                            <path d="M0 0h24v24h-24z" fill="none"></path>
                        </symbol>
                    </defs>
                    {image && (
                        <img src={image} className="map-image" alt="iwi" />
                    )}
                    <use xlinkHref="#pin" x={50} y={55} width="6" height="6" />
                    <text x="50" y="55" class="small" fontSize={8}>The robot is here</text>
                </svg>
            </div>
            <Chart
                width={400}
                height={"300px"}
                chartType="BubbleChart"
                loader={<CircularProgress />}
                data={[
                    [
                        "ID",
                        "Number of Interactions",
                        "Tour Duration",
                        "Tour Id",
                        "Group Size",
                    ],
                    ["3/25", 80.66, 1.67, "3/25", 3],
                    ["3/26", 79.84, 1.36, "3/26", 8],
                    ["3/27", 78.6, 1.84, "3/27", 2],
                    ["3/28", 72.73, 2.78, "3/28", 7],
                    ["3/29", 80.05, 2, "3/29", 6],
                    ["3/30", 72.49, 1.7, "3/30", 7],
                    ["3/31", 68.09, 4.77, "3/31", 3],
                    ["4/1", 81.55, 2.96, "4/1", 17],
                    ["4/2", 68.6, 1.54, "4/2", 14],
                    ["4/3", 78.09, 2.05, "4/3", 30],
                ]}
                options={{
                    title: "Duration of Tour vs Interactions",
                    hAxis: { title: "Number of Interactions" },
                    vAxis: { title: "Duration of Tour" },
                    bubble: { textStyle: { fontSize: 11 } },
                }}
            />
        </div>
    );
}
