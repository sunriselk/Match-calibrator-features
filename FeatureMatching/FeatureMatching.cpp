#include "FeatureMatching.h"
#include <qfiledialog.h>
#include <halconcpp/HalconCpp.h>
#include "EllipseDetection.h"
#include <fstream>
#include <qmessagebox.h>

FeatureMatching::FeatureMatching(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    QObject::connect(ui.PUSHBUTTON_IMAGEDIRECTION, &QPushButton::clicked, this, &FeatureMatching::OnImageDirection);
    QObject::connect(ui.PUSHBUTTON_RESULTDIRECTION, &QPushButton::clicked, this, &FeatureMatching::OnResultDirection);
    QObject::connect(ui.PUSHBUTTON_START, &QPushButton::clicked, this, &FeatureMatching::OnStartMatching);
}

void FeatureMatching::OnImageDirection()
{
    QFileDialog fileDialog(this);
    fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog.setFileMode(QFileDialog::Directory);
    fileDialog.setViewMode(QFileDialog::Detail);


    if (fileDialog.exec())
    {
        m_strImageDir = fileDialog.selectedFiles()[0];
        ui.LINEEDIT_IMAGEDIRECTION->setText(m_strImageDir);
    }
}

void FeatureMatching::OnResultDirection()
{
    QFileDialog fileDialog(this);
    fileDialog.setAcceptMode(QFileDialog::AcceptSave);
    fileDialog.setFileMode(QFileDialog::AnyFile);
    fileDialog.setViewMode(QFileDialog::Detail);
	QString filter = "*.txt";
	fileDialog.setNameFilter(filter);
    if (fileDialog.exec())
    {
        m_strResultDir = fileDialog.selectedFiles()[0];
        ui.LINEEDIT_RESULTDIRECTION->setText(m_strResultDir);
    }
}

void FeatureMatching::OnStartMatching()
{
    QDir dir(m_strImageDir);
    dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
    QStringList strList = dir.entryList();
    QFileInfoList infoList = dir.entryInfoList();

	std::ofstream fout;
	if (m_strResultDir.isEmpty())
	{
		QMessageBox::warning(this, "warning", "No Result Direction!");
		return;
	}
	std::string fullDir = m_strResultDir.toStdString();
	fout.open(fullDir);

    for (int i = 0; i < infoList.size(); i++)
    {
		HalconCpp::HObject  ho_Image;
		HalconCpp::HTuple  hv_ImageFiles, hv_Index;

		ListFiles(infoList[i].absoluteFilePath().toStdString().data(), (HalconCpp::HTuple("files").Append("follow_links")),
			&hv_ImageFiles);

		TupleRegexpSelect(hv_ImageFiles, (HalconCpp::HTuple("\\.(tif|tiff|gif|bmp|jpg|jpeg|jp2|png|pcx|pgm|ppm|pbm|xwd|ima|hobj)$").Append("ignore_case")),
			&hv_ImageFiles);
		HalconCpp::HTuple end_val3 = (hv_ImageFiles.TupleLength()) - 1;
		HalconCpp::HTuple step_val3 = 1;

		std::vector<std::vector<cv::Point2f>> vvImgPoints;
		std::vector<std::vector<cv::Point3f>> vvObjPoints;
		HalconCpp::HTuple nImgWidth, nImgHeight;
		for (hv_Index = 0; hv_Index.Continue(end_val3, step_val3); hv_Index += step_val3)
		{
			ReadImage(&ho_Image, HalconCpp::HTuple(hv_ImageFiles[hv_Index]));

			if (hv_Index == 0)
				HalconCpp::GetImageSize(ho_Image, &nImgWidth, &nImgHeight);

			//´¦ÀíÍ¼Ïñ
			CalibCircleDetection DetectEllipse;
			std::vector<std::pair<int, int>> bigEllipseSortedIndex, outSortIndex;

			bigEllipseSortedIndex.push_back(std::pair<int, int>(8, 5));
			bigEllipseSortedIndex.push_back(std::pair<int, int>(8, 9));
			bigEllipseSortedIndex.push_back(std::pair<int, int>(7, 9));
			bigEllipseSortedIndex.push_back(std::pair<int, int>(5, 7));
			bigEllipseSortedIndex.push_back(std::pair<int, int>(11, 7));

			std::vector<cv::Point2f> sortImageEllipse;
			DetectEllipse.DectectTargetEllipses(ho_Image, 40, 40, 17, 15, bigEllipseSortedIndex, sortImageEllipse, outSortIndex);

			if (0 == sortImageEllipse.size())
				continue;

			cv::Mat cvSrcImg = cv::imread(std::string(hv_ImageFiles[hv_Index]));
			for (uint j = 0; j < sortImageEllipse.size(); j++)
			{
				cv::Scalar color_red = cv::Scalar(0, 0, 255);
				cv::line(cvSrcImg, cv::Point2i(int(sortImageEllipse[j].x) - 5, int(sortImageEllipse[j].y)),
					cv::Point2i(int(sortImageEllipse[j].x) + 5, int(sortImageEllipse[j].y)), color_red);
				cv::line(cvSrcImg, cv::Point2i(int(sortImageEllipse[j].x), int(sortImageEllipse[j].y) - 5),
					cv::Point2i(int(sortImageEllipse[j].x), int(sortImageEllipse[j].y) + 5), color_red);

				std::stringstream ssCode;
				ssCode << 8 << strList[i].toStdString();

				if (outSortIndex[j].first < 10)
					ssCode << 0;
				ssCode << outSortIndex[j].first;
				if (outSortIndex[j].second < 10)
					ssCode << 0;
				ssCode << outSortIndex[j].second;
				std::string strCode;
				ssCode >> strCode;
				cv::putText(cvSrcImg, strCode, cv::Point2i(int(sortImageEllipse[j].x) - 30, int(sortImageEllipse[j].y) - 10),
					cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 0, 255));

				std::string strImageDir = hv_ImageFiles[hv_Index];
				std::string strImageID = strImageDir.substr(strImageDir.length() - 6, 2);
				
				fout << QString(strImageID.data()).toInt() << "\t" << QString(strCode.data()).toInt() << "\t"
					<< sortImageEllipse[j].x << "\t" << sortImageEllipse[j].y << "\t" << 10 << "\t" << 10 << std::endl;
			}
		}
    }
	fout.close();
}
